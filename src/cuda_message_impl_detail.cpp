#include "cuda_message_impl_detail.hpp"

#include "cuda_blackboard/cuda_error.hpp"
#include "cuda_blackboard/cuda_mem_pool_context.hpp"

#include <cuda_runtime_api.h>

namespace cuda_blackboard
{
namespace impl_detail
{

void copyDataToDevice(
  CudaUniquePtr<std::uint8_t[]> & data, cudaEvent_t & ready_event, const void * source_data,
  const std::size_t size, const cudaMemcpyKind copy_kind)
{
  auto & ctx = CudaMemPoolContext::getInstance();

  // NOTE: `cudaEventBlockingSync` flag is not set here so that cudaEventSynchronize()
  // will busy-wait until the event has been completed
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventCreateWithFlags(&ready_event, cudaEventDisableTiming));

  data = make_unique<std::uint8_t[]>(size);
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    cudaMemcpyAsync(data.get(), source_data, size, copy_kind, ctx.stream()));

  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaEventRecord(ready_event, ctx.stream()));
}

}  // namespace impl_detail
}  // namespace cuda_blackboard
