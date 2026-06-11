#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

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

void append_float(std::vector<uint8_t> & data, float value)
{
  const uint8_t * bytes = reinterpret_cast<const uint8_t *>(&value);
  data.insert(data.end(), bytes, bytes + sizeof(float));
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

}  // namespace

class PointCloudDownsampler : public rclcpp::Node
{
public:
  PointCloudDownsampler()
  : Node("pointcloud_downsampler")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rslidar_points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/rslidar_points_web");
    mode_ = declare_parameter<std::string>("mode", "voxel");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    voxel_size_ = std::max(0.001, declare_parameter<double>("voxel_size", 0.10));
    point_stride_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("point_stride", 4)));
    include_intensity_ = declare_parameter<bool>("include_intensity", false);

    if (publish_rate_hz_ > 0.0) {
      min_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<PointCloud2>(output_topic_, qos);
    subscription_ = create_subscription<PointCloud2>(
      input_topic_, qos, std::bind(&PointCloudDownsampler::on_pointcloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Downsampling %s -> %s, mode=%s, publish_rate_hz=%.3f, voxel_size=%.3f, point_stride=%d, include_intensity=%s",
      input_topic_.c_str(), output_topic_.c_str(), mode_.c_str(), publish_rate_hz_, voxel_size_,
      point_stride_, include_intensity_ ? "true" : "false");
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

    if (msg->point_step == 0 || msg->width == 0 || msg->height == 0 || msg->data.empty()) {
      return;
    }

    auto x_field = find_field(*msg, "x");
    auto y_field = find_field(*msg, "y");
    auto z_field = find_field(*msg, "z");
    if (!x_field || !y_field || !z_field) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 has no x/y/z fields; dropping frame.");
      return;
    }
    auto intensity_field = include_intensity_ ? find_field(*msg, "intensity") : std::nullopt;

    PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.fields = output_fields(static_cast<bool>(intensity_field));
    out.is_bigendian = false;
    out.point_step = intensity_field ? 16 : 12;
    out.is_dense = true;

    if (mode_ == "voxel") {
      build_voxel_cloud(*msg, *x_field, *y_field, *z_field, intensity_field, out);
    } else {
      build_stride_cloud(*msg, *x_field, *y_field, *z_field, intensity_field, out);
    }

    if (out.width == 0) {
      return;
    }

    out.row_step = out.width * out.point_step;
    publisher_->publish(out);
    last_publish_time_ = now;
  }

  std::vector<PointField> output_fields(bool has_intensity) const
  {
    std::vector<PointField> fields;
    fields.reserve(has_intensity ? 4 : 3);
    fields.push_back(make_float_field("x", 0));
    fields.push_back(make_float_field("y", 4));
    fields.push_back(make_float_field("z", 8));
    if (has_intensity) {
      fields.push_back(make_float_field("intensity", 12));
    }
    return fields;
  }

  PointField make_float_field(const std::string & name, uint32_t offset) const
  {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = PointField::FLOAT32;
    field.count = 1;
    return field;
  }

  void build_voxel_cloud(
    const PointCloud2 & msg,
    const FieldReader & x_field,
    const FieldReader & y_field,
    const FieldReader & z_field,
    const std::optional<FieldReader> & intensity_field,
    PointCloud2 & out)
  {
    const uint32_t total_points = msg.width * msg.height;
    std::unordered_set<VoxelKey, VoxelKeyHash> seen_voxels;
    seen_voxels.reserve(std::max<uint32_t>(1024, total_points / 4));
    reserve_output(out, total_points / 4);

    for (uint32_t index = 0; index < total_points; ++index) {
      const std::size_t start = static_cast<std::size_t>(index) * msg.point_step;
      if (start + msg.point_step > msg.data.size()) {
        break;
      }

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!read_xyz(msg, start, x_field, y_field, z_field, x, y, z)) {
        continue;
      }

      const VoxelKey key{
        static_cast<int32_t>(std::floor(x / voxel_size_)),
        static_cast<int32_t>(std::floor(y / voxel_size_)),
        static_cast<int32_t>(std::floor(z / voxel_size_))};
      if (!seen_voxels.insert(key).second) {
        continue;
      }

      append_compact_point(msg, start, x, y, z, intensity_field, out);
    }
  }

  void build_stride_cloud(
    const PointCloud2 & msg,
    const FieldReader & x_field,
    const FieldReader & y_field,
    const FieldReader & z_field,
    const std::optional<FieldReader> & intensity_field,
    PointCloud2 & out)
  {
    const uint32_t total_points = msg.width * msg.height;
    const uint32_t estimated_points = (total_points + point_stride_ - 1) / point_stride_;
    reserve_output(out, estimated_points);

    for (uint32_t index = 0; index < total_points; index += static_cast<uint32_t>(point_stride_)) {
      const std::size_t start = static_cast<std::size_t>(index) * msg.point_step;
      if (start + msg.point_step > msg.data.size()) {
        break;
      }

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!read_xyz(msg, start, x_field, y_field, z_field, x, y, z)) {
        continue;
      }

      append_compact_point(msg, start, x, y, z, intensity_field, out);
    }
  }

  bool read_xyz(
    const PointCloud2 & msg,
    std::size_t start,
    const FieldReader & x_field,
    const FieldReader & y_field,
    const FieldReader & z_field,
    double & x,
    double & y,
    double & z) const
  {
    if (!read_field_as_double(msg.data, start, x_field, msg.is_bigendian, x) ||
      !read_field_as_double(msg.data, start, y_field, msg.is_bigendian, y) ||
      !read_field_as_double(msg.data, start, z_field, msg.is_bigendian, z))
    {
      return false;
    }
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }

  void append_compact_point(
    const PointCloud2 & msg,
    std::size_t start,
    double x,
    double y,
    double z,
    const std::optional<FieldReader> & intensity_field,
    PointCloud2 & out) const
  {
    double intensity = 0.0;
    if (intensity_field) {
      read_field_as_double(msg.data, start, *intensity_field, msg.is_bigendian, intensity);
    }

    append_float(out.data, static_cast<float>(x));
    append_float(out.data, static_cast<float>(y));
    append_float(out.data, static_cast<float>(z));
    if (intensity_field) {
      append_float(out.data, static_cast<float>(intensity));
    }
    ++out.width;
  }

  void reserve_output(PointCloud2 & out, uint32_t estimated_points) const
  {
    if (estimated_points == 0) {
      return;
    }
    out.data.reserve(static_cast<std::size_t>(estimated_points) * out.point_step);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string mode_;
  double publish_rate_hz_ = 4.0;
  double voxel_size_ = 0.10;
  int point_stride_ = 4;
  bool include_intensity_ = false;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration min_period_{0, 0};
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudDownsampler>());
  rclcpp::shutdown();
  return 0;
}
