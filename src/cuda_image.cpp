
#include "cuda_blackboard/cuda_image.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cuda_runtime_api.h>

namespace cuda_blackboard
{

CudaImage::CudaImage(const CudaImage & image) : sensor_msgs::msg::Image(image)
{
  RCLCPP_WARN(
    rclcpp::get_logger("CudaImage"),
    "CudaImage copy constructor called. This should be avoided and is most likely a design error.");

  data = make_unique<uint8_t[]>(image.height * image.step * sizeof(uint8_t));
  cudaMemcpy(
    data.get(), image.data.get(), image.height * image.step * sizeof(uint8_t),
    cudaMemcpyDeviceToDevice);
}

CudaImage::CudaImage(const sensor_msgs::msg::Image & source)
{
  header = source.header;
  encoding = source.encoding;
  height = source.height;
  width = source.width;
  step = source.step;
  is_bigendian = source.is_bigendian;

  data = make_unique<uint8_t[]>(source.height * source.step * sizeof(uint8_t));
  cudaMemcpy(
    data.get(), source.data.data(), source.height * source.step * sizeof(uint8_t),
    cudaMemcpyHostToDevice);
}

CudaImage::~CudaImage()
{
}

}  // namespace cuda_blackboard
