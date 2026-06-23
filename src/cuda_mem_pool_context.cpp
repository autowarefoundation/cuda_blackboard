#include "cuda_blackboard/cuda_mem_pool_context.hpp"

#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"

#include <cstdint>
#include <cstdlib>
#include <string>

namespace
{
size_t get_mem_pool_release_threshold()
{
  constexpr size_t default_threshold_mb = 1024;  // 1GiB
  // cspell:ignore mibi
  auto mibi_byte_to_byte = [](const auto & mib) { return mib * 1024 * 1024; };

  const char * env_value = std::getenv("CUDA_BLACKBOARD_MEM_POOL_RELEASE_THRESHOLD_MB");
  if (env_value == nullptr) {  // environment variable is not set
    return mibi_byte_to_byte(default_threshold_mb);
  }

  try {
    return mibi_byte_to_byte(std::stoull(env_value));
  } catch (const std::exception &) {
    return mibi_byte_to_byte(default_threshold_mb);
  }
}
}  // namespace

namespace cuda_blackboard
{

CudaMemPoolContext::CudaMemPoolContext()
{
  // The dedicated stream to handle this memory pool frees from the default stream operations
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaStreamCreateWithFlags(&free_stream_, cudaStreamNonBlocking));

  int device_id = 0;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaStreamGetDevice(stream_, &device_id));

  cudaMemPoolProps pool_props = {};
  pool_props.allocType = cudaMemAllocationTypePinned;
  pool_props.location.id = device_id;
  pool_props.location.type = cudaMemLocationTypeDevice;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemPoolCreate(&pool_, &pool_props));

  // Configure the memory pool reusing allocation
  // we set a high release threshold so that the allocated memory region will be reused
  uint64_t pool_release_threshold = get_mem_pool_release_threshold();
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
    pool_, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));
}

CudaMemPoolContext & CudaMemPoolContext::getInstance()
{
  static CudaMemPoolContext instance;
  return instance;
}

CudaMemPoolContext::~CudaMemPoolContext()
{
  if (stream_) {
    cudaStreamSynchronize(stream_);
  }
  if (free_stream_) {
    cudaStreamSynchronize(free_stream_);
  }
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
  if (free_stream_) {
    cudaStreamDestroy(free_stream_);
  }
  if (pool_) {
    cudaMemPoolDestroy(pool_);
  }
}

void CudaMemPoolContext::blockCpuUntilStreamCompletion()
{
  cudaEvent_t local_wait_event;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&local_wait_event, cudaEventDisableTiming));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(local_wait_event, stream_));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventSynchronize(local_wait_event));
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventDestroy(local_wait_event));
}

}  // namespace cuda_blackboard
