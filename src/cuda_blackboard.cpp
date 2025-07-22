
#include "cuda_blackboard/cuda_blackboard.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>

namespace cuda_blackboard
{

template <typename T>
CudaBlackboard<T> & CudaBlackboard<T>::getInstance()
{
  static CudaBlackboard instance;
  return instance;
}

template <typename T>
uint64_t CudaBlackboard<T>::registerData(
  const std::string & producer_name, std::unique_ptr<const T> data, std::size_t tickets)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::mt19937_64 gen(rd_());
  std::uniform_int_distribution<uint64_t> dist(0, UINT64_MAX);
  uint64_t instance_id = dist(gen);

  while (instance_id_to_data_map_.count(instance_id) > 0) {
    instance_id = dist(gen);
  }

  std::shared_ptr<CudaBlackboardDataWrapper<T>> data_wrapper =
    std::make_shared<CudaBlackboardDataWrapper<T>>(
      std::move(data), producer_name, instance_id, tickets);

  if (producer_to_data_map_.count(producer_name) > 0) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("CudaBlackboard"),
      "Producer " << producer_name << " already exists. Deleting. It had "
                  << producer_to_data_map_[producer_name]->tickets_ << " tickets left");
    instance_id_to_data_map_.erase(producer_to_data_map_[producer_name]->instance_id_);
    producer_to_data_map_.erase(producer_name);
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("CudaBlackboard"), "Registering data from producer %s with instance id %lu",
    producer_name.c_str(), instance_id);
  producer_to_data_map_[producer_name] = data_wrapper;
  instance_id_to_data_map_[instance_id] = data_wrapper;

  return instance_id;
}

template <typename T>
std::shared_ptr<const T> CudaBlackboard<T>::queryData(const std::string & producer_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = producer_to_data_map_.find(producer_name);

  if (it == producer_to_data_map_.end()) {
    return std::shared_ptr<const T>{};  // Indicate that the key was not found
  }
  
  it->second->tickets_--;
  auto data = it->second->data_ptr_;

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("CudaBlackboard"),
    "Producer " << producer_name << " has " << it->second->tickets_ << " tickets left");

  if (it->second->tickets_ == 0) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("CudaBlackboard"), "Removing data from producer " << producer_name);
    instance_id_to_data_map_.erase(it->second->instance_id_);
    producer_to_data_map_.erase(it);
  }

  return data;
}

template <typename T>
std::shared_ptr<const T> CudaBlackboard<T>::queryData(uint64_t instance_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = instance_id_to_data_map_.find(instance_id);

  if (it == instance_id_to_data_map_.end()) {
    return std::shared_ptr<const T>{};  // Indicate that the key was not found
  }

  it->second->tickets_--;
  std::shared_ptr<const T> data = it->second->data_ptr_;

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("CudaBlackboard"),
    "Instance " << instance_id << " has " << it->second->tickets_ << " tickets left");

  if (it->second->tickets_ == 0) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("CudaBlackboard"), "Removing data from instance " << instance_id);
    producer_to_data_map_.erase(it->second->producer_name_);
    instance_id_to_data_map_.erase(it);
  }

  assert(data);
  return data;
}

}  // namespace cuda_blackboard

template class cuda_blackboard::CudaBlackboard<cuda_blackboard::CudaPointCloud2>;
template class cuda_blackboard::CudaBlackboard<cuda_blackboard::CudaImage>;
