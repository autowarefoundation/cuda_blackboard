#ifndef CUDA_BLACKBOARD__CUDA_MEM_POOL_CONTEXT_HPP_
#define CUDA_BLACKBOARD__CUDA_MEM_POOL_CONTEXT_HPP_

#include <cuda_runtime.h>

namespace cuda_blackboard
{
/**
 * @brief Singleton context that owns the CUDA stream and memory pool used by cuda_blackboard.
 */
class CudaMemPoolContext
{
public:
  /// Returns the process-wide CUDA memory pool context.
  static CudaMemPoolContext & getInstance();

  /// Returns the stream used for asynchronous memory pool operations.
  cudaStream_t stream() { return stream_; }

  /// Returns the CUDA memory pool used for pooled device allocations.
  cudaMemPool_t pool() { return pool_; }

private:
  /// Creates the CUDA stream and memory pool owned by this context.
  CudaMemPoolContext();
  ~CudaMemPoolContext();

  cudaStream_t stream_{nullptr};
  cudaMemPool_t pool_{};

  /// This singleton owns CUDA resources and must not be copied or moved.
  CudaMemPoolContext(const CudaMemPoolContext &) = delete;
  CudaMemPoolContext & operator=(const CudaMemPoolContext &) = delete;
  CudaMemPoolContext(CudaMemPoolContext &&) = delete;
  CudaMemPoolContext & operator=(CudaMemPoolContext &&) = delete;
};
}  // namespace cuda_blackboard

#endif  // CUDA_BLACKBOARD__CUDA_MEM_POOL_CONTEXT_HPP_
