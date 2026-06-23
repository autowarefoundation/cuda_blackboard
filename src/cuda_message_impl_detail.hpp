#ifndef CUDA_MESSAGE_IMPL_DETAIL_HPP_
#define CUDA_MESSAGE_IMPL_DETAIL_HPP_

#include "cuda_blackboard/cuda_unique_ptr.hpp"

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>

namespace cuda_blackboard
{
namespace impl_detail
{

void copyDataToDevice(
  CudaUniquePtr<std::uint8_t[]> & data, cudaEvent_t & ready_event, const void * source_data,
  std::size_t size, cudaMemcpyKind copy_kind);

}  // namespace impl_detail
}  // namespace cuda_blackboard

#endif  // CUDA_MESSAGE_IMPL_DETAIL_HPP_
