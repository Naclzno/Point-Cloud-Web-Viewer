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

struct PointXYZ
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

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

}  // namespace

class PointCloudWsServer : public rclcpp::Node
{
public:
  PointCloudWsServer()
  : Node("pointcloud_ws_server")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    voxel_size_ = std::max(0.001, declare_parameter<double>("voxel_size", 0.10));
    accumulate_map_ = declare_parameter<bool>("accumulate_map", false);
    map_voxel_size_ = std::max(0.001, declare_parameter<double>("map_voxel_size", voxel_size_));
    map_window_seconds_ = std::max(0.0, declare_parameter<double>("map_window_seconds", 0.0));
    address_ = declare_parameter<std::string>("address", "0.0.0.0");
    port_ = static_cast<uint16_t>(std::max<int64_t>(1, declare_parameter<int64_t>("port", 8766)));

    if (publish_rate_hz_ > 0.0) {
      min_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    subscription_ = create_subscription<PointCloud2>(
      input_topic_, qos, std::bind(&PointCloudWsServer::on_pointcloud, this, std::placeholders::_1));

    server_thread_ = std::thread([this]() { run_server(); });

    RCLCPP_INFO(
      get_logger(),
      "Point cloud WebSocket server listening on ws://%s:%u, input_topic=%s, publish_rate_hz=%.3f, voxel_size=%.3f, accumulate_map=%s, map_voxel_size=%.3f, map_window_seconds=%.3f",
      address_.c_str(), port_, input_topic_.c_str(), publish_rate_hz_, voxel_size_,
      accumulate_map_ ? "true" : "false", map_voxel_size_, map_window_seconds_);
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

    for (uint32_t index = 0; index < total_points; ++index) {
      const std::size_t start = static_cast<std::size_t>(index) * msg->point_step;
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
          entry->second.last_seen = now;
        }
      } else {
        append_point(*payload, point);
      }
    }

    if (accumulate_map_) {
      prune_map(now);
      payload = build_map_payload();
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_payload_ = std::move(payload);
      ++frame_sequence_;
    }
    frame_cv_.notify_all();
    last_publish_time_ = now;
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

  void run_session(tcp::socket socket)
  {
    try {
      websocket::stream<tcp::socket> ws(std::move(socket));
      ws.set_option(websocket::stream_base::timeout::suggested(boost::beast::role_type::server));
      ws.accept();
      ws.binary(true);

      std::size_t last_sequence = 0;
      while (!stopping_.load()) {
        std::shared_ptr<const std::vector<uint8_t>> payload;
        {
          std::unique_lock<std::mutex> lock(frame_mutex_);
          frame_cv_.wait(lock, [this, last_sequence]() {
            return stopping_.load() || frame_sequence_ != last_sequence;
          });
          if (stopping_.load()) {
            return;
          }
          payload = latest_payload_;
          last_sequence = frame_sequence_;
        }

        if (payload && !payload->empty()) {
          ws.write(boost::asio::buffer(*payload));
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_INFO(get_logger(), "WebSocket client disconnected: %s", error.what());
    }
  }

  std::string input_topic_;
  std::string address_;
  uint16_t port_ = 8766;
  double publish_rate_hz_ = 5.0;
  double voxel_size_ = 0.10;
  bool accumulate_map_ = false;
  double map_voxel_size_ = 0.10;
  double map_window_seconds_ = 0.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration min_period_{0, 0};
  std::unordered_map<VoxelKey, MapPoint, VoxelKeyHash> map_points_;

  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::thread server_thread_;
  std::atomic_bool stopping_{false};

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::shared_ptr<const std::vector<uint8_t>> latest_payload_;
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
