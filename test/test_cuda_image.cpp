#include "cuda_blackboard/cuda_adaptation.hpp"
#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_test_utils.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace cuda_blackboard::test
{

// Fixture for CudaImage tests: requires a CUDA device (via CudaDeviceTest) and gathers the
// shared ROS-image factory and comparison/copy helpers used across the test cases.
class CudaImageTest : public CudaDeviceTest
{
protected:
  static sensor_msgs::msg::Image MakeRosImage()
  {
    sensor_msgs::msg::Image image;
    image.header.frame_id = "camera";
    image.header.stamp.sec = 12;
    image.header.stamp.nanosec = 34;
    image.height = 2;
    image.width = 3;
    image.encoding = "rgb8";
    image.is_bigendian = false;
    image.step = image.width * 3;
    image.data.resize(image.height * image.step);
    for (std::size_t i = 0; i < image.data.size(); ++i) {
      image.data[i] = static_cast<uint8_t>((i + 1) % 256);
    }
    return image;
  }

  static void ExpectSameMetadata(
    const sensor_msgs::msg::Image & expected, const sensor_msgs::msg::Image & actual)
  {
    EXPECT_EQ(actual.header.frame_id, expected.header.frame_id);
    EXPECT_EQ(actual.header.stamp.sec, expected.header.stamp.sec);
    EXPECT_EQ(actual.header.stamp.nanosec, expected.header.stamp.nanosec);
    EXPECT_EQ(actual.height, expected.height);
    EXPECT_EQ(actual.width, expected.width);
    EXPECT_EQ(actual.encoding, expected.encoding);
    EXPECT_EQ(actual.is_bigendian, expected.is_bigendian);
    EXPECT_EQ(actual.step, expected.step);
  }

  static std::vector<uint8_t> CopyImageDataToHost(const CudaImage & image)
  {
    std::vector<uint8_t> host(image.height * image.step);
    EXPECT_TRUE(CudaSucceeded(cudaMemcpy(
      host.data(), image.data.get(), host.size() * sizeof(uint8_t), cudaMemcpyDeviceToHost)));
    return host;
  }
};

TEST_F(CudaImageTest, ConstructsFromRosImageAndConvertsBack)
{
  const auto ros_image = MakeRosImage();
  const CudaImage cuda_image(ros_image);

  // Since allocation (cuda_blackboard::make_unique) is synchronous,
  // cuda_image.data.get() always should have valid pointer at this point.
  ASSERT_NE(cuda_image.data.get(), nullptr);
  // memory copy (HostToDevice) in the cuda_image constructor is asynchronous, needs sync here
  EXPECT_TRUE(CudaSucceeded(cudaDeviceSynchronize()));

  sensor_msgs::msg::Image converted;
  rclcpp::TypeAdapter<CudaImage, sensor_msgs::msg::Image>::convert_to_ros_message(
    cuda_image, converted);

  // Type adaptation copy (DeviceToHost) is synchronous
  ExpectSameMetadata(ros_image, converted);
  EXPECT_EQ(converted.data, ros_image.data);
}

TEST_F(CudaImageTest, CopyConstructorDeepCopiesDeviceData)
{
  const auto ros_image = MakeRosImage();
  CudaImage original(ros_image);
  const CudaImage copy(original);
  // memory copy (DeviceToDevice) in the cuda_image constructor is asynchronous, needs sync here
  EXPECT_TRUE(CudaSucceeded(cudaDeviceSynchronize()));

  ASSERT_NE(original.data.get(), nullptr);
  ASSERT_NE(copy.data.get(), nullptr);
  // Copy constructor allocates new region from the pool, so the address should differ from the
  // original
  EXPECT_NE(copy.data.get(), original.data.get());
  ExpectSameMetadata(original, copy);
  EXPECT_EQ(CopyImageDataToHost(copy), ros_image.data);

  std::vector<uint8_t> zeros(ros_image.data.size(), 0);
  EXPECT_TRUE(CudaSucceeded(cudaMemcpy(
    original.data.get(), zeros.data(), zeros.size() * sizeof(uint8_t), cudaMemcpyHostToDevice)));

  // Since the copy constructor creates "deep" copy, modification on original data should not affect
  // copy
  EXPECT_EQ(CopyImageDataToHost(copy), ros_image.data);
  EXPECT_EQ(CopyImageDataToHost(original), zeros);
}

TEST_F(CudaImageTest, TypeAdapterRoundTripPreservesData)
{
  const auto ros_image = MakeRosImage();

  CudaImage cuda_image;
  rclcpp::TypeAdapter<CudaImage, sensor_msgs::msg::Image>::convert_to_custom(ros_image, cuda_image);
  // Since allocation (cuda_blackboard::make_unique) is synchronous,
  // cuda_image.data.get() always should have valid pointer at this point.
  ASSERT_NE(cuda_image.data.get(), nullptr);
  // Type adaptation copy (HostToDevice) is synchronous
  EXPECT_EQ(CopyImageDataToHost(cuda_image), ros_image.data);

  sensor_msgs::msg::Image converted;
  rclcpp::TypeAdapter<CudaImage, sensor_msgs::msg::Image>::convert_to_ros_message(
    cuda_image, converted);

  ExpectSameMetadata(ros_image, converted);
  // Type adaptation copy (DeviceToHost) is synchronous
  EXPECT_EQ(converted.data, ros_image.data);
}

TEST_F(CudaImageTest, MoveConstructorTransfersDeviceData)
{
  const auto ros_image = MakeRosImage();
  CudaImage source(ros_image);
  auto * const device_ptr = source.data.get();

  // move constructor is declared as "default"
  const CudaImage moved(std::move(source));

  EXPECT_EQ(moved.data.get(), device_ptr);
  ExpectSameMetadata(ros_image, moved);
  EXPECT_EQ(CopyImageDataToHost(moved), ros_image.data);
}

TEST_F(CudaImageTest, MoveAssignmentTransfersDeviceData)
{
  const auto ros_image = MakeRosImage();
  CudaImage source(ros_image);
  auto * const device_ptr = source.data.get();

  // move assignment is declared as "default"
  CudaImage moved;
  moved = std::move(source);

  EXPECT_EQ(moved.data.get(), device_ptr);
  ExpectSameMetadata(ros_image, moved);
  EXPECT_EQ(CopyImageDataToHost(moved), ros_image.data);
}

}  // namespace cuda_blackboard::test
