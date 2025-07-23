
#pragma once

#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"

#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>

namespace cuda_blackboard
{

template <typename T>
class CudaBlackboardDataWrapper
{
public:
  CudaBlackboardDataWrapper(
    std::shared_ptr<const T> data_ptr, const std::string producer_name, uint64_t instance_id,
    std::size_t tickets)
  : data_ptr_(data_ptr), producer_name_(producer_name), instance_id_(instance_id), tickets_(tickets)
  {
  }
  std::shared_ptr<const T> data_ptr_;
  std::string producer_name_;
  uint64_t instance_id_;
  std::size_t tickets_;
};

template <typename T>
class CudaBlackboard
{
public:
  using DataUniquePtrConst = std::unique_ptr<const T>;
  using DataPtrConst = std::shared_ptr<const T>;
  using CudaBlackboardDataWrapperPtr = std::shared_ptr<CudaBlackboardDataWrapper<T>>;

  CudaBlackboard(const CudaBlackboard &) = delete;
  CudaBlackboard & operator=(const CudaBlackboard &) = delete;

  CudaBlackboard(CudaBlackboard &&) = delete;
  CudaBlackboard & operator=(CudaBlackboard &&) = delete;

  ~CudaBlackboard() = default;

  static CudaBlackboard & getInstance();

  uint64_t registerData(
    const std::string & producer_name, std::unique_ptr<const T> value, std::size_t tickets);

  std::shared_ptr<const T> queryData(const std::string & producer_name);

  std::shared_ptr<const T> queryData(uint64_t instance_id);

private:
  std::unordered_map<std::string, CudaBlackboardDataWrapperPtr> producer_to_data_map_;
  std::unordered_map<uint64_t, CudaBlackboardDataWrapperPtr> instance_id_to_data_map_;
  std::mutex mutex_;

  CudaBlackboard() {}

  void reset();

  std::random_device rd_;
};

}  // namespace cuda_blackboard
