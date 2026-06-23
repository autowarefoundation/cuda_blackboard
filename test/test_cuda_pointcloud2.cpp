#include "cuda_blackboard/cuda_adaptation.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"
#include "cuda_test_utils.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace cuda_blackboard::test
{

// Fixture for CudaPointCloud2 tests: requires a CUDA device (via CudaDeviceTest) and gathers the
// shared ROS-pointcloud factory and comparison/copy helpers used across the test cases.
class CudaPointCloud2Test : public CudaDeviceTest
{
protected:
  static sensor_msgs::msg::PointCloud2 MakeRosPointCloud2()
  {
    sensor_msgs::msg::PointCloud2 pointcloud;
    pointcloud.header.frame_id = "lidar";
    pointcloud.header.stamp.sec = 56;
    pointcloud.header.stamp.nanosec = 78;
    pointcloud.height = 2;
    pointcloud.width = 3;
    pointcloud.is_bigendian = false;
    pointcloud.point_step = 4;
    pointcloud.row_step = pointcloud.width * pointcloud.point_step;
    pointcloud.is_dense = true;

    sensor_msgs::msg::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    pointcloud.fields.push_back(field);

    pointcloud.data.resize(pointcloud.height * pointcloud.row_step);
    for (std::size_t i = 0; i < pointcloud.data.size(); ++i) {
      pointcloud.data[i] = static_cast<uint8_t>(i + 11);
    }
    return pointcloud;
  }

  static void ExpectSameMetadata(
    const sensor_msgs::msg::PointCloud2 & expected, const sensor_msgs::msg::PointCloud2 & actual)
  {
    EXPECT_EQ(actual.header.frame_id, expected.header.frame_id);
    EXPECT_EQ(actual.header.stamp.sec, expected.header.stamp.sec);
    EXPECT_EQ(actual.header.stamp.nanosec, expected.header.stamp.nanosec);
    EXPECT_EQ(actual.height, expected.height);
    EXPECT_EQ(actual.width, expected.width);
    ASSERT_EQ(actual.fields.size(), expected.fields.size());
    EXPECT_EQ(actual.fields[0].name, expected.fields[0].name);
    EXPECT_EQ(actual.fields[0].offset, expected.fields[0].offset);
    EXPECT_EQ(actual.fields[0].datatype, expected.fields[0].datatype);
    EXPECT_EQ(actual.fields[0].count, expected.fields[0].count);
    EXPECT_EQ(actual.is_bigendian, expected.is_bigendian);
    EXPECT_EQ(actual.point_step, expected.point_step);
    EXPECT_EQ(actual.row_step, expected.row_step);
    EXPECT_EQ(actual.is_dense, expected.is_dense);
  }

  static std::vector<uint8_t> CopyPointCloudDataToHost(const CudaPointCloud2 & pointcloud)
  {
    std::vector<uint8_t> host(pointcloud.height * pointcloud.width * pointcloud.point_step);
    EXPECT_TRUE(CudaSucceeded(cudaMemcpy(
      host.data(), pointcloud.data.get(), host.size() * sizeof(uint8_t), cudaMemcpyDeviceToHost)));
    return host;
  }
};

TEST_F(CudaPointCloud2Test, ConstructsFromRosPointCloudAndConvertsBack)
{
  const auto ros_pointcloud = MakeRosPointCloud2();
  const CudaPointCloud2 cuda_pointcloud(ros_pointcloud);

  // Since allocation (cuda_blackboard::make_unique) is synchronous,
  // cuda_pointcloud.data.get() always should have valid pointer at this point.
  ASSERT_NE(cuda_pointcloud.data.get(), nullptr);
  // memory copy (HostToDevice) in the cuda_pointcloud constructor is asynchronous, needs sync here
  EXPECT_TRUE(CudaSucceeded(cudaDeviceSynchronize()));

  sensor_msgs::msg::PointCloud2 converted;
  rclcpp::TypeAdapter<CudaPointCloud2, sensor_msgs::msg::PointCloud2>::convert_to_ros_message(
    cuda_pointcloud, converted);

  // Type adaptation copy (DeviceToHost) is synchronous
  ExpectSameMetadata(ros_pointcloud, converted);
  EXPECT_EQ(converted.data, ros_pointcloud.data);
}

TEST_F(CudaPointCloud2Test, CopyConstructorDeepCopiesDeviceData)
{
  const auto ros_pointcloud = MakeRosPointCloud2();
  CudaPointCloud2 original(ros_pointcloud);
  const CudaPointCloud2 copy(original);
  // memory copy (DeviceToDevice) in the cuda_pointcloud constructor is asynchronous, needs sync
  // here
  EXPECT_TRUE(CudaSucceeded(cudaDeviceSynchronize()));

  ASSERT_NE(original.data.get(), nullptr);
  ASSERT_NE(copy.data.get(), nullptr);
  // Copy constructor allocates new region from the pool, so the address should differ from the
  // original
  EXPECT_NE(copy.data.get(), original.data.get());
  ExpectSameMetadata(original, copy);
  EXPECT_EQ(CopyPointCloudDataToHost(copy), ros_pointcloud.data);

  std::vector<uint8_t> zeros(ros_pointcloud.data.size(), 0);
  EXPECT_TRUE(CudaSucceeded(cudaMemcpy(
    original.data.get(), zeros.data(), zeros.size() * sizeof(uint8_t), cudaMemcpyHostToDevice)));

  // Since the copy constructor creates "deep" copy, modification on original data should not affect
  // copy
  EXPECT_EQ(CopyPointCloudDataToHost(copy), ros_pointcloud.data);
  EXPECT_EQ(CopyPointCloudDataToHost(original), zeros);
}

TEST_F(CudaPointCloud2Test, TypeAdapterRoundTripPreservesData)
{
  const auto ros_pointcloud = MakeRosPointCloud2();

  CudaPointCloud2 cuda_pointcloud;
  rclcpp::TypeAdapter<CudaPointCloud2, sensor_msgs::msg::PointCloud2>::convert_to_custom(
    ros_pointcloud, cuda_pointcloud);
  // Since allocation (cuda_blackboard::make_unique) is synchronous,
  // cuda_pointcloud.data.get() always should have valid pointer at this point.
  ASSERT_NE(cuda_pointcloud.data.get(), nullptr);
  EXPECT_EQ(CopyPointCloudDataToHost(cuda_pointcloud), ros_pointcloud.data);

  sensor_msgs::msg::PointCloud2 converted;
  rclcpp::TypeAdapter<CudaPointCloud2, sensor_msgs::msg::PointCloud2>::convert_to_ros_message(
    cuda_pointcloud, converted);

  ExpectSameMetadata(ros_pointcloud, converted);
  // Type adaptation copy (DeviceToHost) is synchronous
  EXPECT_EQ(converted.data, ros_pointcloud.data);
}

TEST_F(CudaPointCloud2Test, MoveConstructorTransfersOwnership)
{
  const auto ros_pointcloud = MakeRosPointCloud2();
  CudaPointCloud2 source(ros_pointcloud);
  auto * const device_ptr = source.data.get();

  // Dedicated move constructor moves source.data to moved.data
  CudaPointCloud2 moved(std::move(source));

  EXPECT_EQ(source.data.get(), nullptr);
  EXPECT_EQ(moved.data.get(), device_ptr);
  ExpectSameMetadata(ros_pointcloud, moved);
  EXPECT_EQ(CopyPointCloudDataToHost(moved), ros_pointcloud.data);
}

TEST_F(CudaPointCloud2Test, MoveAssignmentTransfersOwnership)
{
  const auto ros_pointcloud = MakeRosPointCloud2();
  CudaPointCloud2 source(ros_pointcloud);
  auto * const device_ptr = source.data.get();

  // Dedicated move assignment moves source.data to moved.data
  CudaPointCloud2 moved;
  moved = std::move(source);

  EXPECT_EQ(source.data.get(), nullptr);
  EXPECT_EQ(moved.data.get(), device_ptr);
  ExpectSameMetadata(ros_pointcloud, moved);
  EXPECT_EQ(CopyPointCloudDataToHost(moved), ros_pointcloud.data);
}

}  // namespace cuda_blackboard::test
