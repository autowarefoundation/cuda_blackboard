#ifndef CUDA_BLACKBOARD__CUDA_ERROR_HPP_
#define CUDA_BLACKBOARD__CUDA_ERROR_HPP_

#include <cuda_runtime_api.h>

#include <sstream>
#include <stdexcept>

#define CUDA_BLACKBOARD_CHECK_CUDA_ERROR(e) \
  (cuda_blackboard::cuda_check_error(e, __FILE__, __LINE__))

namespace cuda_blackboard
{

template <typename F, typename N>
void cuda_check_error(const ::cudaError_t e, F && f, N && n)
{
  if (e != ::cudaSuccess) {
    std::stringstream s;
    s << ::cudaGetErrorName(e) << " (" << e << ")@" << f << "#L" << n << ": "
      << ::cudaGetErrorString(e);
    throw std::runtime_error{s.str()};
  }
}

}  // namespace cuda_blackboard

#endif  // CUDA_BLACKBOARD__CUDA_ERROR_HPP_
