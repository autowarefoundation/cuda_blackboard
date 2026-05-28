
#pragma once

#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"

#include <cuda_runtime_api.h>

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
    std::size_t tickets, cudaEvent_t ready_event = nullptr)
  : data_ptr_(data_ptr),
    producer_name_(producer_name),
    instance_id_(instance_id),
    tickets_(tickets),
    ready_event_(ready_event)
  {
  }

  CudaBlackboardDataWrapper(const CudaBlackboardDataWrapper &) = delete;
  CudaBlackboardDataWrapper & operator=(const CudaBlackboardDataWrapper &) = delete;

  ~CudaBlackboardDataWrapper()
  {
    // Owns the "ready" event the publisher recorded on its stream. The wrapper
    // is kept alive (via aliasing shared_ptrs handed out by queryData) until
    // every subscriber has queued its wait on the event, so destroying it here
    // is safe. The buffer in data_ptr_ is released right after, on the consumer
    // stream when one was set on its deleter.
    if (ready_event_ != nullptr) {
      cudaEventDestroy(ready_event_);
    }
  }

  std::shared_ptr<const T> data_ptr_;
  std::string producer_name_;
  uint64_t instance_id_;
  std::size_t tickets_;
  cudaEvent_t ready_event_{nullptr};
};

template <typename T>
class CudaBlackboard
{
public:
  using DataUniquePtrConst = std::unique_ptr<const T>;
  using DataPtrConst = std::shared_ptr<const T>;
  using CudaBlackboardDataWrapperPtr = std::shared_ptr<CudaBlackboardDataWrapper<T>>;

  static CudaBlackboard & getInstance();

  uint64_t registerData(
    const std::string & producer_name, std::unique_ptr<const T> value, std::size_t tickets,
    cudaEvent_t ready_event = nullptr);
  std::shared_ptr<const T> queryData(const std::string & producer_name);
  // When ready_event_out is non-null, it receives the publisher's "ready" event
  // for this instance so the subscriber can order its stream after the fill.
  std::shared_ptr<const T> queryData(uint64_t instance_id, cudaEvent_t * ready_event_out = nullptr);

private:
  std::unordered_map<std::string, CudaBlackboardDataWrapperPtr> producer_to_data_map_;
  std::unordered_map<uint64_t, CudaBlackboardDataWrapperPtr> instance_id_to_data_map_;
  std::mutex mutex_;

  CudaBlackboard() {}
  CudaBlackboard(const CudaBlackboard &) = delete;
  CudaBlackboard & operator=(const CudaBlackboard &) = delete;

  std::random_device rd_;
};

}  // namespace cuda_blackboard
