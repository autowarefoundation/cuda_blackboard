
#pragma once

#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_blackboard/cuda_mem_pool_context.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"
#include "cuda_blackboard/cuda_unique_ptr.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/type_adapter.hpp>

#include <cuda_runtime_api.h>

#include <cstdint>

template <>
struct rclcpp::TypeAdapter<cuda_blackboard::CudaImage, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = cuda_blackboard::CudaImage;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.header = source.header;
    destination.encoding = source.encoding;
    destination.height = source.height;
    destination.width = source.width;
    destination.step = source.step;
    destination.is_bigendian = source.is_bigendian;

    auto & ctx = cuda_blackboard::CudaMemPoolContext::getInstance();

    RCLCPP_DEBUG(rclcpp::get_logger("CudaImage"), "Converting to ROS message");
    destination.data.resize(source.height * source.step * sizeof(uint8_t));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemcpyAsync(
      destination.data.data(), source.data.get(), source.height * source.step * sizeof(uint8_t),
      cudaMemcpyDeviceToHost, ctx.stream()));

    // Block CPU thread until operation finishes
    cudaEvent_t local_wait_event;
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
      cudaEventCreateWithFlags(&local_wait_event, cudaEventDisableTiming));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(local_wait_event, ctx.stream()));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventSynchronize(local_wait_event));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventDestroy(local_wait_event));
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.header = source.header;
    destination.encoding = source.encoding;
    destination.height = source.height;
    destination.width = source.width;
    destination.step = source.step;
    destination.is_bigendian = source.is_bigendian;

    auto & ctx = cuda_blackboard::CudaMemPoolContext::getInstance();

    destination.data = cuda_blackboard::make_unique<uint8_t[]>(source.height * source.step);
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemcpyAsync(
      destination.data.get(), source.data.data(), source.height * source.step * sizeof(uint8_t),
      cudaMemcpyHostToDevice, ctx.stream()));

    // Block CPU thread until operation finishes
    cudaEvent_t local_wait_event;
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
      cudaEventCreateWithFlags(&local_wait_event, cudaEventDisableTiming));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(local_wait_event, ctx.stream()));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventSynchronize(local_wait_event));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventDestroy(local_wait_event));
  }
};

template <>
struct rclcpp::TypeAdapter<cuda_blackboard::CudaPointCloud2, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = cuda_blackboard::CudaPointCloud2;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.header = source.header;
    destination.height = source.height;
    destination.width = source.width;
    destination.fields = source.fields;
    destination.is_bigendian = source.is_bigendian;
    destination.point_step = source.point_step;
    destination.row_step = source.row_step;
    destination.is_dense = source.is_dense;

    auto & ctx = cuda_blackboard::CudaMemPoolContext::getInstance();

    destination.data.resize(source.height * source.width * source.point_step);
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemcpyAsync(
      destination.data.data(), source.data.get(),
      source.height * source.width * source.point_step * sizeof(uint8_t), cudaMemcpyDeviceToHost,
      ctx.stream()));

    // Block CPU thread until operation finishes
    cudaEvent_t local_wait_event;
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
      cudaEventCreateWithFlags(&local_wait_event, cudaEventDisableTiming));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(local_wait_event, ctx.stream()));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventSynchronize(local_wait_event));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventDestroy(local_wait_event));
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.header = source.header;
    destination.height = source.height;
    destination.width = source.width;
    destination.fields = source.fields;
    destination.is_bigendian = source.is_bigendian;
    destination.point_step = source.point_step;
    destination.row_step = source.row_step;
    destination.is_dense = source.is_dense;

    auto & ctx = cuda_blackboard::CudaMemPoolContext::getInstance();

    RCLCPP_DEBUG(rclcpp::get_logger("CudaPointCloud2"), "Converting from ROS message");
    destination.data =
      cuda_blackboard::make_unique<uint8_t[]>(source.height * source.width * source.point_step);
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemcpyAsync(
      destination.data.get(), source.data.data(),
      source.height * source.width * source.point_step * sizeof(uint8_t), cudaMemcpyHostToDevice,
      ctx.stream()));

    // Block CPU thread until operation finishes
    cudaEvent_t local_wait_event;
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
      cudaEventCreateWithFlags(&local_wait_event, cudaEventDisableTiming));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(local_wait_event, ctx.stream()));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventSynchronize(local_wait_event));
    CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventDestroy(local_wait_event));
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(cuda_blackboard::CudaImage, sensor_msgs::msg::Image);

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  cuda_blackboard::CudaPointCloud2, sensor_msgs::msg::PointCloud2);
