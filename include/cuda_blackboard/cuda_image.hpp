

#pragma once

#include "cuda_blackboard/cuda_unique_ptr.hpp"

#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <memory>

namespace cuda_blackboard
{

class CudaImage : public sensor_msgs::msg::Image
{
public:
  using ros_type = sensor_msgs::msg::Image;
  using SharedPtr =
    std::shared_ptr<CudaImage>;
  using ConstSharedPtr =
    std::shared_ptr<CudaImage const>;

  template<typename Deleter = std::default_delete<
      CudaImage>>
  using UniquePtrWithDeleter =
    std::unique_ptr<CudaImage, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      CudaImage>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<CudaImage const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;
 
  CudaImage() = default;
  CudaImage(const CudaImage & image);  // This is needed for ROS compliance
  CudaImage & operator=(const CudaImage &) = delete;
  CudaImage(CudaImage && image) = default;
  CudaImage & operator=(CudaImage &&) = default;
  CudaImage(const sensor_msgs::msg::Image & source);

  ~CudaImage();

  CudaUniquePtr<std::uint8_t[]> data;
};

}  // namespace cuda_blackboard
