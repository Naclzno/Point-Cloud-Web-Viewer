#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "patchworkpp/patchworkpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using PatchworkPoint = pcl::PointXYZI;

namespace
{

struct FieldReader
{
  uint32_t offset = 0;
  uint8_t datatype = 0;
};

struct VoxelKey
{
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = 0;
    seed ^= std::hash<int32_t>{}(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int32_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int32_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct GridKey
{
  int32_t x = 0;
  int32_t y = 0;

  bool operator==(const GridKey & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey & key) const
  {
    std::size_t seed = 0;
    seed ^= std::hash<int32_t>{}(key.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int32_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct PointXYZ
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct LocalPoint
{
  double x = 0.0;
  double y = 0.0;
  double height = 0.0;
};

std::array<double, 3> normalize(std::array<double, 3> value)
{
  const double norm = std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
  if (norm <= 0.0) {
    return {0.0, 0.0, 1.0};
  }
  return {value[0] / norm, value[1] / norm, value[2] / norm};
}

std::array<double, 3> cross(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return {
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]};
}

double dot(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

struct MapPoint
{
  PointXYZ point;
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};

struct MapFrame
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::vector<PointXYZ> points;
};

bool host_is_bigendian()
{
  static const bool is_bigendian = []() {
    const uint16_t value = 0x0102;
    return *reinterpret_cast<const uint8_t *>(&value) == 0x01;
  }();
  return is_bigendian;
}

template<typename T>
T read_scalar(const uint8_t * data, bool data_bigendian)
{
  std::array<uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), data, sizeof(T));
  if (data_bigendian != host_is_bigendian()) {
    std::reverse(bytes.begin(), bytes.end());
  }

  T value{};
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

bool read_field_as_double(
  const std::vector<uint8_t> & data,
  std::size_t start,
  const FieldReader & field,
  bool is_bigendian,
  double & value)
{
  const uint8_t * ptr = data.data() + start + field.offset;
  switch (field.datatype) {
    case PointField::INT8:
      value = static_cast<double>(*reinterpret_cast<const int8_t *>(ptr));
      return true;
    case PointField::UINT8:
      value = static_cast<double>(*ptr);
      return true;
    case PointField::INT16:
      value = static_cast<double>(read_scalar<int16_t>(ptr, is_bigendian));
      return true;
    case PointField::UINT16:
      value = static_cast<double>(read_scalar<uint16_t>(ptr, is_bigendian));
      return true;
    case PointField::INT32:
      value = static_cast<double>(read_scalar<int32_t>(ptr, is_bigendian));
      return true;
    case PointField::UINT32:
      value = static_cast<double>(read_scalar<uint32_t>(ptr, is_bigendian));
      return true;
    case PointField::FLOAT32:
      value = static_cast<double>(read_scalar<float>(ptr, is_bigendian));
      return true;
    case PointField::FLOAT64:
      value = read_scalar<double>(ptr, is_bigendian);
      return true;
    default:
      return false;
  }
}

std::optional<FieldReader> find_field(const PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name && field.count == 1) {
      return FieldReader{field.offset, field.datatype};
    }
  }
  return std::nullopt;
}

void append_float(std::vector<uint8_t> & data, float value)
{
  const uint8_t * bytes = reinterpret_cast<const uint8_t *>(&value);
  data.insert(data.end(), bytes, bytes + sizeof(float));
}

template<typename T>
void write_little_endian(std::ostream & stream, T value)
{
  std::array<uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  if (host_is_bigendian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  stream.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

void write_fixed_string(std::ostream & stream, const std::string & value, std::size_t width)
{
  std::vector<char> buffer(width, '\0');
  const auto copy_size = std::min(width, value.size());
  std::copy_n(value.data(), copy_size, buffer.data());
  stream.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));
}

std::optional<double> json_number(const std::string & json, const std::string & key)
{
  const std::regex pattern(
    "\"" + key + R"("\s*:\s*(-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?))");
  std::smatch match;
  if (!std::regex_search(json, match, pattern) || match.size() < 2) {
    return std::nullopt;
  }
  try {
    return std::stod(match[1].str());
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<std::string> json_string(const std::string & json, const std::string & key)
{
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (!std::regex_search(json, match, pattern) || match.size() < 2) {
    return std::nullopt;
  }
  return match[1].str();
}

bool json_bool(const std::string & json, const std::string & key, bool fallback)
{
  const std::regex pattern("\"" + key + R"("\s*:\s*(true|false))");
  std::smatch match;
  if (!std::regex_search(json, match, pattern) || match.size() < 2) {
    return fallback;
  }
  return match[1].str() == "true";
}

bool solve_3x3(std::array<std::array<double, 4>, 3> matrix, std::array<double, 3> & result)
{
  for (std::size_t col = 0; col < 3; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < 3; ++row) {
      if (std::abs(matrix[row][col]) > std::abs(matrix[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][col]) < 1e-9) {
      return false;
    }
    if (pivot != col) {
      std::swap(matrix[pivot], matrix[col]);
    }

    const double scale = matrix[col][col];
    for (std::size_t item = col; item < 4; ++item) {
      matrix[col][item] /= scale;
    }

    for (std::size_t row = 0; row < 3; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = matrix[row][col];
      for (std::size_t item = col; item < 4; ++item) {
        matrix[row][item] -= factor * matrix[col][item];
      }
    }
  }

  result = {matrix[0][3], matrix[1][3], matrix[2][3]};
  return true;
}

}  // namespace

class PointCloudWsServer : public rclcpp::Node
{
public:
  PointCloudWsServer()
  : Node("pointcloud_ws_server")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    input_qos_ = declare_parameter<std::string>("input_qos", "sensor_data");
    ground_fit_min_points_ = std::max<int64_t>(30, declare_parameter<int64_t>("ground_fit_min_points", 200));
    ground_fit_max_points_ = std::max<int64_t>(ground_fit_min_points_, declare_parameter<int64_t>("ground_fit_max_points", 20000));
    patchwork_update_hz_ = std::max(0.1, declare_parameter<double>("patchwork_update_hz", 2.0));
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    voxel_size_ = std::max(0.001, declare_parameter<double>("voxel_size", 0.10));
    accumulate_map_ = declare_parameter<bool>("accumulate_map", false);
    map_voxel_size_ = std::max(0.001, declare_parameter<double>("map_voxel_size", voxel_size_));
    map_window_seconds_ = std::max(0.0, declare_parameter<double>("map_window_seconds", 0.0));
    enable_volume_ = declare_parameter<bool>("enable_volume", false);
    volume_update_hz_ = std::max(0.001, declare_parameter<double>("volume_update_hz", 1.0));
    volume_min_height_ = std::max(0.0, declare_parameter<double>("volume_min_height", 0.05));
    volume_roi_min_x_ = declare_parameter<double>("volume_roi_min_x", -1000.0);
    volume_roi_max_x_ = declare_parameter<double>("volume_roi_max_x", 1000.0);
    volume_roi_min_y_ = declare_parameter<double>("volume_roi_min_y", -1000.0);
    volume_roi_max_y_ = declare_parameter<double>("volume_roi_max_y", 1000.0);
    las_save_duration_ = std::max(0.0, declare_parameter<double>("las_save_duration", 0.0));
    las_save_path_ = declare_parameter<std::string>("las_save_path", "");
    las_save_enabled_ = las_save_duration_ > 0.0 && !las_save_path_.empty();
    ground_plane_normal_ = normalize(
      {declare_parameter<double>("ground_plane_a", 0.0),
        declare_parameter<double>("ground_plane_b", 0.0),
        declare_parameter<double>("ground_plane_c", 1.0)});
    ground_plane_d_ = declare_parameter<double>("ground_plane_d", 0.0);
    address_ = declare_parameter<std::string>("address", "0.0.0.0");
    port_ = static_cast<uint16_t>(std::max<int64_t>(1, declare_parameter<int64_t>("port", 8766)));

    if (publish_rate_hz_ > 0.0) {
      min_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);
    }
    patchwork_min_period_ = rclcpp::Duration::from_seconds(1.0 / patchwork_update_hz_);
    volume_min_period_ = rclcpp::Duration::from_seconds(1.0 / volume_update_hz_);
    compute_ground_basis();
    patchworkpp_ground_seg_ = std::make_shared<PatchWorkpp<PatchworkPoint>>(this);

    if (las_save_duration_ > 0.0 && las_save_path_.empty()) {
      RCLCPP_WARN(get_logger(), "las_save_duration is set but las_save_path is empty; LAS saving disabled.");
    }
    if (las_save_enabled_) {
      RCLCPP_INFO(
        get_logger(), "LAS capture enabled: duration=%.3fs, path=%s",
        las_save_duration_, las_save_path_.c_str());
    }

    if (enable_volume_ && !accumulate_map_) {
      RCLCPP_WARN(
        get_logger(), "enable_volume=true requires accumulated map data; forcing accumulate_map=true.");
      accumulate_map_ = true;
    }

    if (input_qos_ == "default" || input_qos_ == "reliable") {
      subscription_ = create_subscription<PointCloud2>(
        input_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&PointCloudWsServer::on_pointcloud, this, std::placeholders::_1));
    } else {
      input_qos_ = "sensor_data";
      subscription_ = create_subscription<PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&PointCloudWsServer::on_pointcloud, this, std::placeholders::_1));
    }

    server_thread_ = std::thread([this]() { run_server(); });

    RCLCPP_INFO(
      get_logger(),
      "Point cloud WebSocket server listening on ws://%s:%u, input_topic=%s, input_qos=%s, publish_rate_hz=%.3f, voxel_size=%.3f, accumulate_map=%s, map_voxel_size=%.3f, map_window_seconds=%.3f, enable_volume=%s",
      address_.c_str(), port_, input_topic_.c_str(), input_qos_.c_str(), publish_rate_hz_, voxel_size_,
      accumulate_map_ ? "true" : "false", map_voxel_size_, map_window_seconds_,
      enable_volume_ ? "true" : "false");
  }

  ~PointCloudWsServer() override
  {
    stopping_.store(true);
    frame_cv_.notify_all();
    flush_las_capture_on_shutdown();

    if (acceptor_) {
      boost::system::error_code ec;
      acceptor_->close(ec);
    }
    if (io_context_) {
      io_context_->stop();
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  bool fit_plane_from_ground_points(
    const std::vector<PointXYZ> & points,
    std::array<double, 3> & normal,
    double & plane_d) const
  {
    if (points.size() < static_cast<std::size_t>(ground_fit_min_points_)) {
      return false;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double sum_yy = 0.0;
    double sum_xz = 0.0;
    double sum_yz = 0.0;
    for (const auto & point : points) {
      const double x = point.x;
      const double y = point.y;
      const double z = point.z;
      sum_x += x;
      sum_y += y;
      sum_z += z;
      sum_xx += x * x;
      sum_xy += x * y;
      sum_yy += y * y;
      sum_xz += x * z;
      sum_yz += y * z;
    }

    const double n = static_cast<double>(points.size());
    std::array<double, 3> coefficients{};
    if (!solve_3x3(
        {{{sum_xx, sum_xy, sum_x, sum_xz},
          {sum_xy, sum_yy, sum_y, sum_yz},
          {sum_x, sum_y, n, sum_z}}},
        coefficients))
    {
      return false;
    }

    normal = normalize({-coefficients[0], -coefficients[1], 1.0});
    plane_d = -coefficients[2] /
      std::sqrt(coefficients[0] * coefficients[0] + coefficients[1] * coefficients[1] + 1.0);
    if (normal[2] < 0.0) {
      normal = {-normal[0], -normal[1], -normal[2]};
      plane_d = -plane_d;
    }
    return std::isfinite(normal[0]) && std::isfinite(normal[1]) && std::isfinite(normal[2]) &&
      std::isfinite(plane_d);
  }

  void append_labeled_point(std::vector<uint8_t> & data, const PointXYZ & point, float label) const
  {
    append_float(data, point.x);
    append_float(data, point.y);
    append_float(data, point.z);
    append_float(data, label);
  }

  std::vector<PointXYZ> current_map_points_snapshot() const
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::vector<PointXYZ> points;
    points.reserve(map_points_.size());
    for (const auto & entry : map_points_) {
      points.push_back(entry.second.point);
    }
    return points;
  }

  bool run_patchwork_on_current_window(bool flip_points)
  {
    const auto input_points = current_map_points_snapshot();
    return run_patchwork_on_points(input_points, flip_points, "patchworkpp");
  }

  bool run_patchwork_on_points(
    const std::vector<PointXYZ> & input_points,
    bool flip_points,
    const std::string & source)
  {
    if (input_points.size() < static_cast<std::size_t>(ground_fit_min_points_)) {
      RCLCPP_WARN(
        get_logger(), "Patchwork++ fit failed: only %zu accumulated points available.",
        input_points.size());
      publish_patchwork_status(
        "error",
        "Patchwork++ failed: not enough accumulated sliding-window points");
      return false;
    }

    pcl::PointCloud<PatchworkPoint> input_cloud;
    input_cloud.reserve(input_points.size());
    for (const auto & point : input_points) {
      PatchworkPoint pcl_point;
      pcl_point.x = point.x;
      pcl_point.y = flip_points ? -point.y : point.y;
      pcl_point.z = flip_points ? -point.z : point.z;
      pcl_point.intensity = 0.0F;
      input_cloud.push_back(pcl_point);
    }

    pcl::PointCloud<PatchworkPoint> ground_cloud;
    pcl::PointCloud<PatchworkPoint> nonground_cloud;
    double time_taken = 0.0;
    patchworkpp_ground_seg_->estimate_ground(input_cloud, ground_cloud, nonground_cloud, time_taken);

    std::vector<PointXYZ> ground_points;
    ground_points.reserve(ground_cloud.size());
    for (const auto & point : ground_cloud.points) {
      ground_points.push_back(PointXYZ{point.x, point.y, point.z});
    }

    std::array<double, 3> normal{};
    double plane_d = 0.0;
    if (!fit_plane_from_ground_points(ground_points, normal, plane_d)) {
      RCLCPP_WARN(
        get_logger(), "Patchwork++ ground plane fit failed from %zu ground points.",
        ground_points.size());
      publish_patchwork_status(
        "error",
        "Patchwork++ failed: could not fit a ground plane from segmented ground points");
      return false;
    }

    {
      std::lock_guard<std::mutex> config_lock(config_mutex_);
      ground_plane_normal_ = normal;
      ground_plane_d_ = plane_d;
      compute_ground_basis();
    }

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json.precision(6);
    json << "{\"type\":\"patchwork_result\",\"source\":\"" << source << "\""
         << ",\"ground_a\":" << normal[0]
         << ",\"ground_b\":" << normal[1]
         << ",\"ground_c\":" << normal[2]
         << ",\"ground_d\":" << plane_d
         << ",\"input_points\":" << input_points.size()
         << ",\"ground_points\":" << ground_cloud.size()
         << ",\"nonground_points\":" << nonground_cloud.size()
         << ",\"time_sec\":" << time_taken
         << "}";

    auto payload = std::make_shared<std::vector<uint8_t>>();
    payload->reserve((ground_cloud.size() + nonground_cloud.size()) * 4 * sizeof(float));
    for (const auto & point : ground_cloud.points) {
      append_labeled_point(*payload, PointXYZ{point.x, point.y, point.z}, 0.0F);
    }
    for (const auto & point : nonground_cloud.points) {
      append_labeled_point(*payload, PointXYZ{point.x, point.y, point.z}, 1.0F);
    }

    {
      std::lock_guard<std::mutex> frame_lock(frame_mutex_);
      latest_segmentation_json_ = json.str();
      latest_segmentation_payload_ = std::move(payload);
      ++segmentation_sequence_;
      ++frame_sequence_;
    }
    frame_cv_.notify_all();

    RCLCPP_INFO(
      get_logger(),
      "Patchwork++ %s segmentation: input=%zu ground=%zu nonground=%zu plane=%.6f %.6f %.6f %.6f time=%.6fs.",
      source.c_str(), input_points.size(), ground_cloud.size(), nonground_cloud.size(), normal[0],
      normal[1], normal[2], plane_d, time_taken);
    return true;
  }

  void maybe_run_patchwork_stream(const rclcpp::Time & now)
  {
    bool enabled = false;
    bool flip_points = false;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      enabled = patchwork_stream_enabled_;
      flip_points = patchwork_flip_points_;
    }
    if (!enabled) {
      return;
    }
    if (last_patchwork_time_.nanoseconds() != 0 && now - last_patchwork_time_ < patchwork_min_period_) {
      return;
    }
    last_patchwork_time_ = now;
    run_patchwork_on_current_window(flip_points);
  }

  void publish_patchwork_status(const std::string & level, const std::string & message)
  {
    std::ostringstream json;
    json << "{\"type\":\"patchwork_status\",\"level\":\"" << level << "\",\"message\":\"";
    for (const char ch : message) {
      if (ch == '"' || ch == '\\') {
        json << '\\';
      }
      json << ch;
    }
    json << "\"}";
    {
      std::lock_guard<std::mutex> frame_lock(frame_mutex_);
      latest_segmentation_json_ = json.str();
      latest_segmentation_payload_.reset();
      ++segmentation_sequence_;
      ++frame_sequence_;
    }
    frame_cv_.notify_all();
  }

  std::filesystem::path resolve_las_output_path(const rclcpp::Time & now) const
  {
    std::filesystem::path path(las_save_path_);
    if (path.extension() == ".las" || path.extension() == ".LAS") {
      return path;
    }
    const auto stamp = now.seconds();
    const auto sec = static_cast<int64_t>(std::floor(stamp));
    const auto nsec = static_cast<int64_t>((stamp - static_cast<double>(sec)) * 1e9);
    std::ostringstream filename;
    filename << "pointcloud_" << sec << "_" << nsec << ".las";
    return path / filename.str();
  }

  bool write_las_file(const std::filesystem::path & output_path, const std::vector<PointXYZ> & points)
  {
    if (points.empty()) {
      RCLCPP_WARN(get_logger(), "LAS capture finished with no points; no file written.");
      return false;
    }
    if (points.size() > static_cast<std::size_t>(std::numeric_limits<uint32_t>::max())) {
      RCLCPP_ERROR(
        get_logger(), "LAS capture has %zu points, exceeding LAS 1.2 point count limit.",
        points.size());
      return false;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto & point : points) {
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
      max_z = std::max(max_z, static_cast<double>(point.z));
    }

    std::error_code fs_error;
    const auto parent = output_path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent, fs_error);
      if (fs_error) {
        RCLCPP_ERROR(
          get_logger(), "Failed to create LAS output directory %s: %s",
          parent.string().c_str(), fs_error.message().c_str());
        return false;
      }
    }

    std::ofstream file(output_path, std::ios::binary);
    if (!file) {
      RCLCPP_ERROR(get_logger(), "Failed to open LAS output file: %s", output_path.string().c_str());
      return false;
    }

    constexpr uint16_t header_size = 227;
    constexpr uint32_t point_data_offset = 227;
    constexpr uint8_t point_format = 0;
    constexpr uint16_t point_record_length = 20;
    constexpr double scale = 0.001;
    const double offset_x = min_x;
    const double offset_y = min_y;
    const double offset_z = min_z;
    const auto point_count = static_cast<uint32_t>(points.size());

    file.write("LASF", 4);
    write_little_endian<uint16_t>(file, 0);
    write_little_endian<uint16_t>(file, 0);
    write_little_endian<uint32_t>(file, 0);
    write_little_endian<uint16_t>(file, 0);
    write_little_endian<uint16_t>(file, 0);
    for (int i = 0; i < 8; ++i) {
      write_little_endian<uint8_t>(file, 0);
    }
    write_little_endian<uint8_t>(file, 1);
    write_little_endian<uint8_t>(file, 2);
    write_fixed_string(file, "pointcloud_web_tools", 32);
    write_fixed_string(file, "pointcloud_ws_server", 32);
    write_little_endian<uint16_t>(file, 1);
    write_little_endian<uint16_t>(file, 2026);
    write_little_endian<uint16_t>(file, header_size);
    write_little_endian<uint32_t>(file, point_data_offset);
    write_little_endian<uint32_t>(file, 0);
    write_little_endian<uint8_t>(file, point_format);
    write_little_endian<uint16_t>(file, point_record_length);
    write_little_endian<uint32_t>(file, point_count);
    write_little_endian<uint32_t>(file, point_count);
    for (int i = 1; i < 5; ++i) {
      write_little_endian<uint32_t>(file, 0);
    }
    write_little_endian<double>(file, scale);
    write_little_endian<double>(file, scale);
    write_little_endian<double>(file, scale);
    write_little_endian<double>(file, offset_x);
    write_little_endian<double>(file, offset_y);
    write_little_endian<double>(file, offset_z);
    write_little_endian<double>(file, max_x);
    write_little_endian<double>(file, min_x);
    write_little_endian<double>(file, max_y);
    write_little_endian<double>(file, min_y);
    write_little_endian<double>(file, max_z);
    write_little_endian<double>(file, min_z);

    for (const auto & point : points) {
      const auto ix = static_cast<int32_t>(std::llround((static_cast<double>(point.x) - offset_x) / scale));
      const auto iy = static_cast<int32_t>(std::llround((static_cast<double>(point.y) - offset_y) / scale));
      const auto iz = static_cast<int32_t>(std::llround((static_cast<double>(point.z) - offset_z) / scale));
      write_little_endian<int32_t>(file, ix);
      write_little_endian<int32_t>(file, iy);
      write_little_endian<int32_t>(file, iz);
      write_little_endian<uint16_t>(file, 0);
      write_little_endian<uint8_t>(file, 0);
      write_little_endian<uint8_t>(file, 1);
      write_little_endian<int8_t>(file, 0);
      write_little_endian<uint8_t>(file, 0);
      write_little_endian<uint16_t>(file, 0);
    }

    if (!file) {
      RCLCPP_ERROR(get_logger(), "Failed while writing LAS output file: %s", output_path.string().c_str());
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "Saved LAS point cloud: %s (%zu points)",
      output_path.string().c_str(), points.size());
    return true;
  }

  void update_las_capture(const rclcpp::Time & now, const std::vector<PointXYZ> & frame_points)
  {
    if (las_capture_complete_) {
      return;
    }
    if (las_capture_start_time_.nanoseconds() == 0) {
      las_capture_start_time_ = now;
      las_capture_points_.clear();
      RCLCPP_INFO(
        get_logger(), "Started LAS capture for %.3fs to %s",
        las_save_duration_, las_save_path_.c_str());
    }

    las_capture_points_.insert(las_capture_points_.end(), frame_points.begin(), frame_points.end());
    if (now - las_capture_start_time_ < rclcpp::Duration::from_seconds(las_save_duration_)) {
      return;
    }

    const auto output_path = resolve_las_output_path(now);
    write_las_file(output_path, las_capture_points_);
    las_capture_complete_ = true;
    las_capture_points_.clear();
  }

  void flush_las_capture_on_shutdown()
  {
    if (!las_save_enabled_ || las_capture_complete_ || las_capture_points_.empty()) {
      return;
    }
    const auto now = get_clock()->now();
    const auto output_path = resolve_las_output_path(now);
    RCLCPP_WARN(
      get_logger(),
      "LAS capture stopped before %.3fs; saving partial capture with %zu points.",
      las_save_duration_, las_capture_points_.size());
    write_las_file(output_path, las_capture_points_);
    las_capture_complete_ = true;
    las_capture_points_.clear();
  }

  void on_pointcloud(const PointCloud2::SharedPtr msg)
  {
    const auto now = get_clock()->now();
    if (last_publish_time_.nanoseconds() != 0 && min_period_.nanoseconds() > 0 &&
      now - last_publish_time_ < min_period_)
    {
      return;
    }

    auto x_field = find_field(*msg, "x");
    auto y_field = find_field(*msg, "y");
    auto z_field = find_field(*msg, "z");
    if (!x_field || !y_field || !z_field || msg->point_step == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 has no readable x/y/z fields.");
      return;
    }

    const uint32_t total_points = msg->width * msg->height;
    std::unordered_set<VoxelKey, VoxelKeyHash> seen_voxels;
    seen_voxels.reserve(std::max<uint32_t>(1024, total_points / 4));

    auto payload = std::make_shared<std::vector<uint8_t>>();
    payload->reserve(static_cast<std::size_t>(std::max<uint32_t>(1024, total_points / 4)) * 3 * sizeof(float));
    std::vector<PointXYZ> frame_points;
    if (accumulate_map_) {
      frame_points.reserve(std::max<uint32_t>(1024, total_points / 4));
    }
    std::vector<PointXYZ> raw_las_points;
    if (las_save_enabled_) {
      raw_las_points.reserve(total_points);
    }

    for (uint32_t row = 0; row < msg->height; ++row) {
      const std::size_t row_start = static_cast<std::size_t>(row) * msg->row_step;
      for (uint32_t col = 0; col < msg->width; ++col) {
        const std::size_t start = row_start + static_cast<std::size_t>(col) * msg->point_step;
        if (start + msg->point_step > msg->data.size()) {
          break;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!read_field_as_double(msg->data, start, *x_field, msg->is_bigendian, x) ||
          !read_field_as_double(msg->data, start, *y_field, msg->is_bigendian, y) ||
          !read_field_as_double(msg->data, start, *z_field, msg->is_bigendian, z) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
          continue;
        }

        const PointXYZ raw_point{
          static_cast<float>(x),
          static_cast<float>(y),
          static_cast<float>(z)};
        if (las_save_enabled_) {
          raw_las_points.push_back(raw_point);
        }

        const VoxelKey key{
          static_cast<int32_t>(std::floor(x / voxel_size_)),
          static_cast<int32_t>(std::floor(y / voxel_size_)),
          static_cast<int32_t>(std::floor(z / voxel_size_))};
        if (!seen_voxels.insert(key).second) {
          continue;
        }

        const PointXYZ point = raw_point;
        if (accumulate_map_) {
          frame_points.push_back(point);
        }
        if (!accumulate_map_) {
          append_point(*payload, point);
        }
      }
    }

    if (las_save_enabled_) {
      update_las_capture(now, raw_las_points);
    }

    if (accumulate_map_) {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_frames_.push_back(MapFrame{now, std::move(frame_points)});
      prune_map_frames(now);
      rebuild_map_from_frames();
      maybe_update_volume(now);
      payload = build_map_payload();
    }
    maybe_run_patchwork_stream(now);
    const auto payload_points = payload ? payload->size() / (3 * sizeof(float)) : 0U;
    const auto map_points = [this]() {
      std::lock_guard<std::mutex> lock(map_mutex_);
      return map_points_.size();
    }();

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_payload_ = std::move(payload);
      if (pending_volume_json_) {
        latest_volume_json_ = *pending_volume_json_;
        pending_volume_json_.reset();
      }
      ++frame_sequence_;
    }
    frame_cv_.notify_all();
    last_publish_time_ = now;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received %u points from %s; web payload points=%zu; accumulated map points=%zu; window frames=%zu.",
      total_points, input_topic_.c_str(), payload_points, map_points, map_frames_.size());
  }

  void run_server()
  {
    try {
      io_context_ = std::make_unique<boost::asio::io_context>();
      auto endpoint = tcp::endpoint(boost::asio::ip::make_address(address_), port_);
      acceptor_ = std::make_unique<tcp::acceptor>(*io_context_);
      acceptor_->open(endpoint.protocol());
      acceptor_->set_option(boost::asio::socket_base::reuse_address(true));
      acceptor_->bind(endpoint);
      acceptor_->listen(boost::asio::socket_base::max_listen_connections);

      while (!stopping_.load()) {
        boost::system::error_code ec;
        tcp::socket socket(*io_context_);
        acceptor_->accept(socket, ec);
        if (ec) {
          if (!stopping_.load()) {
            RCLCPP_WARN(get_logger(), "WebSocket accept failed: %s", ec.message().c_str());
          }
          continue;
        }

        std::thread(&PointCloudWsServer::run_session, this, std::move(socket)).detach();
      }
    } catch (const std::exception & error) {
      if (!stopping_.load()) {
        RCLCPP_ERROR(get_logger(), "WebSocket server stopped: %s", error.what());
      }
    }
  }

  void append_point(std::vector<uint8_t> & data, const PointXYZ & point) const
  {
    append_float(data, point.x);
    append_float(data, point.y);
    append_float(data, point.z);
  }

  std::shared_ptr<std::vector<uint8_t>> build_map_payload() const
  {
    auto payload = std::make_shared<std::vector<uint8_t>>();
    payload->reserve(map_points_.size() * 3 * sizeof(float));
    for (const auto & entry : map_points_) {
      append_point(*payload, entry.second.point);
    }
    return payload;
  }

  void prune_map_frames(const rclcpp::Time & now)
  {
    if (map_window_seconds_ <= 0.0) {
      return;
    }

    const auto window = rclcpp::Duration::from_seconds(map_window_seconds_);
    while (!map_frames_.empty() && now - map_frames_.front().stamp > window) {
      map_frames_.pop_front();
    }
  }

  void rebuild_map_from_frames()
  {
    map_points_.clear();
    for (const auto & frame : map_frames_) {
      for (const auto & point : frame.points) {
        const VoxelKey map_key{
          static_cast<int32_t>(std::floor(point.x / map_voxel_size_)),
          static_cast<int32_t>(std::floor(point.y / map_voxel_size_)),
          static_cast<int32_t>(std::floor(point.z / map_voxel_size_))};
        map_points_[map_key] = MapPoint{point, frame.stamp};
      }
    }
  }

  void compute_ground_basis()
  {
    std::array<double, 3> helper{0.0, 0.0, 1.0};
    if (std::abs(dot(ground_plane_normal_, helper)) > 0.95) {
      helper = {1.0, 0.0, 0.0};
    }
    ground_x_axis_ = normalize(cross(helper, ground_plane_normal_));
    ground_y_axis_ = normalize(cross(ground_plane_normal_, ground_x_axis_));
    ground_origin_ = {
      -ground_plane_d_ * ground_plane_normal_[0],
      -ground_plane_d_ * ground_plane_normal_[1],
      -ground_plane_d_ * ground_plane_normal_[2]};
  }

  LocalPoint to_ground_coordinates(const PointXYZ & point) const
  {
    const std::array<double, 3> relative{
      static_cast<double>(point.x) - ground_origin_[0],
      static_cast<double>(point.y) - ground_origin_[1],
      static_cast<double>(point.z) - ground_origin_[2]};
    return {dot(relative, ground_x_axis_), dot(relative, ground_y_axis_), dot(relative, ground_plane_normal_)};
  }

  void maybe_update_volume(const rclcpp::Time & now)
  {
    std::lock_guard<std::mutex> config_lock(config_mutex_);
    if (!enable_volume_) {
      return;
    }
    if (last_volume_time_.nanoseconds() != 0 && now - last_volume_time_ < volume_min_period_) {
      return;
    }

    std::unordered_map<GridKey, double, GridKeyHash> height_grid;
    height_grid.reserve(map_points_.size());

    for (const auto & entry : map_points_) {
      const LocalPoint local = to_ground_coordinates(entry.second.point);
      if (entry.second.point.x < volume_roi_min_x_ || entry.second.point.x > volume_roi_max_x_ ||
        entry.second.point.y < volume_roi_min_y_ || entry.second.point.y > volume_roi_max_y_ ||
        local.height <= volume_min_height_)
      {
        continue;
      }

      const GridKey key{
        static_cast<int32_t>(std::floor(entry.second.point.x / map_voxel_size_)),
        static_cast<int32_t>(std::floor(entry.second.point.y / map_voxel_size_))};
      auto existing = height_grid.find(key);
      if (existing == height_grid.end() || local.height > existing->second) {
        height_grid[key] = local.height;
      }
    }

    const double cell_area = map_voxel_size_ * map_voxel_size_;
    double volume = 0.0;
    for (const auto & cell : height_grid) {
      volume += cell.second * cell_area;
    }

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json.precision(4);
    json << "{\"type\":\"volume\",\"volume_m3\":" << volume
         << ",\"cells\":" << height_grid.size()
         << ",\"map_points\":" << map_points_.size()
         << ",\"grid_size\":" << map_voxel_size_
         << ",\"window_seconds\":" << map_window_seconds_
         << ",\"source\":\"server\"}";
    pending_volume_json_ = json.str();
    last_volume_time_ = now;
  }

  std::optional<bool> stream_command_enabled(const std::string & json) const
  {
    const auto type = json_string(json, "type");
    if (!type || *type != "stream") {
      return std::nullopt;
    }
    return json_bool(json, "enabled", false);
  }

  bool is_fit_ground_command(const std::string & json) const
  {
    const auto type = json_string(json, "type");
    return type && *type == "fit_ground";
  }

  bool is_patchwork_stream_command(const std::string & json) const
  {
    const auto type = json_string(json, "type");
    return type && *type == "patchwork_stream";
  }

  bool is_patchwork_static_upload_command(const std::string & json) const
  {
    const auto type = json_string(json, "type");
    return type && *type == "patchwork_static_upload";
  }

  void apply_patchwork_stream_command(const std::string & json)
  {
    const bool enabled = json_bool(json, "enabled", false);
    const bool flip_points = json_bool(json, "flip", false);
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      patchwork_stream_enabled_ = enabled;
      patchwork_flip_points_ = flip_points;
    }
    last_patchwork_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    RCLCPP_INFO(
      get_logger(), "Patchwork++ realtime segmentation %s with flip=%s.",
      enabled ? "enabled" : "disabled", flip_points ? "true" : "false");
    if (enabled) {
      run_patchwork_on_current_window(flip_points);
    } else {
      publish_patchwork_status("info", "Patchwork++ realtime segmentation disabled");
    }
  }

  bool run_patchwork_static_upload(const boost::beast::flat_buffer & buffer, bool flip_points)
  {
    const auto bytes = boost::beast::buffers_to_string(buffer.data());
    if (bytes.size() < 3 * sizeof(float) || bytes.size() % (3 * sizeof(float)) != 0) {
      publish_patchwork_status("error", "Static Patchwork++ upload failed: invalid binary point buffer");
      return false;
    }

    const auto point_count = bytes.size() / (3 * sizeof(float));
    std::vector<PointXYZ> points;
    points.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      PointXYZ point;
      std::memcpy(&point.x, bytes.data() + index * 3 * sizeof(float), sizeof(float));
      std::memcpy(&point.y, bytes.data() + (index * 3 + 1) * sizeof(float), sizeof(float));
      std::memcpy(&point.z, bytes.data() + (index * 3 + 2) * sizeof(float), sizeof(float));
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points.push_back(point);
      }
    }

    RCLCPP_INFO(get_logger(), "Received static Patchwork++ upload: %zu points.", points.size());
    return run_patchwork_on_points(points, flip_points, "patchworkpp_static");
  }

  bool apply_client_command(const std::string & json)
  {
    if (json.find("\"type\"") == std::string::npos ||
      json.find("calibration") == std::string::npos)
    {
      return false;
    }

    const auto ground_a = json_number(json, "ground_a");
    const auto ground_b = json_number(json, "ground_b");
    const auto ground_c = json_number(json, "ground_c");
    const auto ground_d = json_number(json, "ground_d");
    const auto roi_min_x = json_number(json, "roi_min_x");
    const auto roi_max_x = json_number(json, "roi_max_x");
    const auto roi_min_y = json_number(json, "roi_min_y");
    const auto roi_max_y = json_number(json, "roi_max_y");
    const bool has_roi = json_bool(json, "has_roi", false);

    std::lock_guard<std::mutex> lock(config_mutex_);
    if (ground_a && ground_b && ground_c && ground_d) {
      ground_plane_normal_ = normalize({*ground_a, *ground_b, *ground_c});
      ground_plane_d_ = *ground_d;
      compute_ground_basis();
      RCLCPP_INFO(
        get_logger(), "Updated ground plane from web: %.6f %.6f %.6f %.6f",
        ground_plane_normal_[0], ground_plane_normal_[1], ground_plane_normal_[2],
        ground_plane_d_);
    }

    if (has_roi && roi_min_x && roi_max_x && roi_min_y && roi_max_y) {
      volume_roi_min_x_ = std::min(*roi_min_x, *roi_max_x);
      volume_roi_max_x_ = std::max(*roi_min_x, *roi_max_x);
      volume_roi_min_y_ = std::min(*roi_min_y, *roi_max_y);
      volume_roi_max_y_ = std::max(*roi_min_y, *roi_max_y);
      enable_volume_ = true;
      if (!accumulate_map_) {
        accumulate_map_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Received ROI from web with accumulate_map=false; enabling accumulation for volume.");
      }
      last_volume_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      RCLCPP_INFO(
        get_logger(), "Updated ROI from web: x=[%.3f, %.3f], y=[%.3f, %.3f]",
        volume_roi_min_x_, volume_roi_max_x_, volume_roi_min_y_, volume_roi_max_y_);
    } else if (!has_roi) {
      enable_volume_ = false;
      last_volume_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      {
        std::lock_guard<std::mutex> frame_lock(frame_mutex_);
        latest_volume_json_.clear();
        pending_volume_json_.reset();
        ++frame_sequence_;
      }
      frame_cv_.notify_all();
      RCLCPP_INFO(get_logger(), "Cleared ROI from web; disabled volume output.");
    }

    return true;
  }

  bool read_pending_client_messages(
    websocket::stream<tcp::socket> & ws,
    bool & stream_enabled,
    bool & stream_state_changed,
    bool & expect_static_patchwork_upload,
    bool & static_patchwork_flip,
    bool & segmentation_reply_enabled)
  {
    boost::system::error_code ec;
    const auto available = ws.next_layer().available(ec);
    if (ec || available == 0) {
      return true;
    }

    boost::beast::flat_buffer buffer;
    ws.read(buffer, ec);
    if (ec) {
      return false;
    }
    if (ws.got_text()) {
      const auto message = boost::beast::buffers_to_string(buffer.data());
      if (const auto enabled = stream_command_enabled(message)) {
        stream_state_changed = stream_enabled != *enabled;
        stream_enabled = *enabled;
        RCLCPP_INFO(
          get_logger(), "WebSocket client %s point-cloud stream.",
          stream_enabled ? "enabled" : "disabled");
      } else if (is_patchwork_stream_command(message)) {
        apply_patchwork_stream_command(message);
      } else if (is_patchwork_static_upload_command(message)) {
        static_patchwork_flip = json_bool(message, "flip", false);
        expect_static_patchwork_upload = true;
        segmentation_reply_enabled = true;
        publish_patchwork_status("info", "Static Patchwork++ upload ready");
      } else if (is_fit_ground_command(message)) {
        const bool flip_points = json_bool(message, "flip", false);
        const bool ok = run_patchwork_on_current_window(flip_points);
        RCLCPP_INFO(
          get_logger(), "WebSocket fit_ground command %s with flip=%s.",
          ok ? "completed" : "failed", flip_points ? "true" : "false");
      } else {
        apply_client_command(message);
      }
    } else if (expect_static_patchwork_upload) {
      expect_static_patchwork_upload = false;
      segmentation_reply_enabled = true;
      run_patchwork_static_upload(buffer, static_patchwork_flip);
    }
    return true;
  }

  void run_session(tcp::socket socket)
  {
    try {
      websocket::stream<tcp::socket> ws(std::move(socket));
      ws.set_option(websocket::stream_base::timeout::suggested(boost::beast::role_type::server));
      ws.read_message_max(2ULL * 1024ULL * 1024ULL * 1024ULL);
      ws.accept();
      ws.binary(true);

      std::size_t last_sequence = 0;
      std::size_t last_segmentation_sequence = 0;
      bool stream_enabled = false;
      bool expect_static_patchwork_upload = false;
      bool static_patchwork_flip = false;
      bool segmentation_reply_enabled = false;
      while (!stopping_.load()) {
        std::shared_ptr<const std::vector<uint8_t>> payload;
        std::shared_ptr<const std::vector<uint8_t>> segmentation_payload;
        std::string volume_json;
        std::string segmentation_json;
        bool has_new_frame = false;
        bool has_new_segmentation = false;
        bool stream_state_changed = false;
        {
          std::unique_lock<std::mutex> lock(frame_mutex_);
          frame_cv_.wait_for(lock, std::chrono::milliseconds(20), [this, last_sequence, last_segmentation_sequence]() {
            return stopping_.load() || frame_sequence_ != last_sequence ||
              segmentation_sequence_ != last_segmentation_sequence;
          });
          if (stopping_.load()) {
            return;
          }
          if (segmentation_sequence_ != last_segmentation_sequence) {
            segmentation_payload = latest_segmentation_payload_;
            segmentation_json = latest_segmentation_json_;
            last_segmentation_sequence = segmentation_sequence_;
            has_new_segmentation = !segmentation_json.empty() ||
              static_cast<bool>(segmentation_payload && !segmentation_payload->empty());
          }
          if (frame_sequence_ != last_sequence) {
            payload = latest_payload_;
            volume_json = latest_volume_json_;
            last_sequence = frame_sequence_;
            has_new_frame = true;
          }
        }

        if (!read_pending_client_messages(
            ws, stream_enabled, stream_state_changed, expect_static_patchwork_upload,
            static_patchwork_flip, segmentation_reply_enabled))
        {
          return;
        }

        if (stream_enabled && stream_state_changed) {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          payload = latest_payload_;
          volume_json = latest_volume_json_;
          has_new_frame = static_cast<bool>(payload && !payload->empty());
          RCLCPP_INFO(
            get_logger(), "WebSocket stream enabled; latest payload has %zu bytes.",
            payload ? payload->size() : 0U);
        }

        if (!has_new_frame) {
          continue;
        }

        if (stream_enabled && !volume_json.empty()) {
          ws.text(true);
          ws.write(boost::asio::buffer(volume_json));
        }
        const bool can_send_segmentation = stream_enabled || segmentation_reply_enabled;
        if (can_send_segmentation && has_new_segmentation && !segmentation_json.empty()) {
          ws.text(true);
          ws.write(boost::asio::buffer(segmentation_json));
        }
        if (can_send_segmentation && has_new_segmentation && segmentation_payload && !segmentation_payload->empty()) {
          ws.binary(true);
          ws.write(boost::asio::buffer(*segmentation_payload));
          if (!stream_enabled) {
            segmentation_reply_enabled = false;
          }
        }
        if (stream_enabled && payload && !payload->empty()) {
          ws.binary(true);
          ws.write(boost::asio::buffer(*payload));
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_INFO(get_logger(), "WebSocket client disconnected: %s", error.what());
    }
  }

  std::string input_topic_;
  std::string input_qos_;
  int64_t ground_fit_min_points_ = 200;
  int64_t ground_fit_max_points_ = 20000;
  double patchwork_update_hz_ = 2.0;
  bool patchwork_stream_enabled_ = false;
  bool patchwork_flip_points_ = false;
  std::string address_;
  uint16_t port_ = 8766;
  double publish_rate_hz_ = 5.0;
  double voxel_size_ = 0.10;
  bool accumulate_map_ = false;
  double map_voxel_size_ = 0.10;
  double map_window_seconds_ = 0.0;
  bool enable_volume_ = false;
  double volume_update_hz_ = 1.0;
  double volume_min_height_ = 0.05;
  double volume_roi_min_x_ = -1000.0;
  double volume_roi_max_x_ = 1000.0;
  double volume_roi_min_y_ = -1000.0;
  double volume_roi_max_y_ = 1000.0;
  double las_save_duration_ = 0.0;
  std::string las_save_path_;
  bool las_save_enabled_ = false;
  bool las_capture_complete_ = false;
  double ground_plane_d_ = 0.0;
  std::array<double, 3> ground_plane_normal_{0.0, 0.0, 1.0};
  std::array<double, 3> ground_x_axis_{1.0, 0.0, 0.0};
  std::array<double, 3> ground_y_axis_{0.0, 1.0, 0.0};
  std::array<double, 3> ground_origin_{0.0, 0.0, 0.0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_volume_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_patchwork_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time las_capture_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration min_period_{0, 0};
  rclcpp::Duration volume_min_period_{1, 0};
  rclcpp::Duration patchwork_min_period_{0, 0};
  std::deque<MapFrame> map_frames_;
  std::unordered_map<VoxelKey, MapPoint, VoxelKeyHash> map_points_;
  std::vector<PointXYZ> las_capture_points_;
  mutable std::mutex map_mutex_;
  std::mutex config_mutex_;
  std::shared_ptr<PatchWorkpp<PatchworkPoint>> patchworkpp_ground_seg_;

  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::thread server_thread_;
  std::atomic_bool stopping_{false};

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::shared_ptr<const std::vector<uint8_t>> latest_payload_;
  std::string latest_volume_json_;
  std::shared_ptr<const std::vector<uint8_t>> latest_segmentation_payload_;
  std::string latest_segmentation_json_;
  std::optional<std::string> pending_volume_json_;
  std::size_t frame_sequence_ = 0;
  std::size_t segmentation_sequence_ = 0;

  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudWsServer>());
  rclcpp::shutdown();
  return 0;
}
