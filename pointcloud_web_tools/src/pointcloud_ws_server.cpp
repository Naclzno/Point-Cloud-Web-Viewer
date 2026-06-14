#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

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

bool json_bool(const std::string & json, const std::string & key, bool fallback)
{
  const std::regex pattern("\"" + key + R"("\s*:\s*(true|false))");
  std::smatch match;
  if (!std::regex_search(json, match, pattern) || match.size() < 2) {
    return fallback;
  }
  return match[1].str() == "true";
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
    volume_min_period_ = rclcpp::Duration::from_seconds(1.0 / volume_update_hz_);
    compute_ground_basis();

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

        const VoxelKey key{
          static_cast<int32_t>(std::floor(x / voxel_size_)),
          static_cast<int32_t>(std::floor(y / voxel_size_)),
          static_cast<int32_t>(std::floor(z / voxel_size_))};
        if (!seen_voxels.insert(key).second) {
          continue;
        }

        const PointXYZ point{
          static_cast<float>(x),
          static_cast<float>(y),
          static_cast<float>(z)};
        if (accumulate_map_) {
          const VoxelKey map_key{
            static_cast<int32_t>(std::floor(x / map_voxel_size_)),
            static_cast<int32_t>(std::floor(y / map_voxel_size_)),
            static_cast<int32_t>(std::floor(z / map_voxel_size_))};
          auto [entry, inserted] = map_points_.try_emplace(map_key, MapPoint{point, now});
          if (!inserted) {
            entry->second.point = point;
            entry->second.last_seen = now;
          }
        } else {
          append_point(*payload, point);
        }
      }
    }

    if (accumulate_map_) {
      prune_map(now);
      maybe_update_volume(now);
      payload = build_map_payload();
    }
    const auto payload_points = payload ? payload->size() / (3 * sizeof(float)) : 0U;
    const auto map_points = map_points_.size();

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
      "Received %u points from %s; web payload points=%zu; accumulated map points=%zu.",
      total_points, input_topic_.c_str(), payload_points, map_points);
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

  void prune_map(const rclcpp::Time & now)
  {
    if (map_window_seconds_ <= 0.0) {
      return;
    }

    const auto window = rclcpp::Duration::from_seconds(map_window_seconds_);
    for (auto it = map_points_.begin(); it != map_points_.end();) {
      if (now - it->second.last_seen > window) {
        it = map_points_.erase(it);
      } else {
        ++it;
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
    if (json.find("\"type\"") == std::string::npos || json.find("stream") == std::string::npos) {
      return std::nullopt;
    }
    return json_bool(json, "enabled", false);
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
    bool & stream_state_changed)
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
      } else {
        apply_client_command(message);
      }
    }
    return true;
  }

  void run_session(tcp::socket socket)
  {
    try {
      websocket::stream<tcp::socket> ws(std::move(socket));
      ws.set_option(websocket::stream_base::timeout::suggested(boost::beast::role_type::server));
      ws.accept();
      ws.binary(true);

      std::size_t last_sequence = 0;
      bool stream_enabled = false;
      while (!stopping_.load()) {
        std::shared_ptr<const std::vector<uint8_t>> payload;
        std::string volume_json;
        bool has_new_frame = false;
        bool stream_state_changed = false;
        {
          std::unique_lock<std::mutex> lock(frame_mutex_);
          has_new_frame = frame_cv_.wait_for(lock, std::chrono::milliseconds(20), [this, last_sequence]() {
            return stopping_.load() || frame_sequence_ != last_sequence;
          });
          if (stopping_.load()) {
            return;
          }
          if (has_new_frame) {
            payload = latest_payload_;
            volume_json = latest_volume_json_;
            last_sequence = frame_sequence_;
          }
        }

        if (!read_pending_client_messages(ws, stream_enabled, stream_state_changed)) {
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
  double ground_plane_d_ = 0.0;
  std::array<double, 3> ground_plane_normal_{0.0, 0.0, 1.0};
  std::array<double, 3> ground_x_axis_{1.0, 0.0, 0.0};
  std::array<double, 3> ground_y_axis_{0.0, 1.0, 0.0};
  std::array<double, 3> ground_origin_{0.0, 0.0, 0.0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_volume_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration min_period_{0, 0};
  rclcpp::Duration volume_min_period_{1, 0};
  std::unordered_map<VoxelKey, MapPoint, VoxelKeyHash> map_points_;
  std::mutex config_mutex_;

  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::thread server_thread_;
  std::atomic_bool stopping_{false};

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::shared_ptr<const std::vector<uint8_t>> latest_payload_;
  std::string latest_volume_json_;
  std::optional<std::string> pending_volume_json_;
  std::size_t frame_sequence_ = 0;

  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudWsServer>());
  rclcpp::shutdown();
  return 0;
}
