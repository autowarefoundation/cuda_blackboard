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

  /// Returns the stream used for every memory pool operation, allocation and free alike.
  ///
  /// INVARIANT: allocations (cudaMallocFromPoolAsync), frees (cudaFreeAsync) and the
  /// consumer-completion waits injected by CudaBlackboardSubscriber all go through this one
  /// stream. Stream-ordered allocation then guarantees that a block freed here is immediately
  /// reusable by the next allocation, with no event plumbing and no chance of the allocator
  /// conservatively reserving new memory because it cannot prove the free happened first.
  cudaStream_t stream() { return stream_; }

  /// Returns the CUDA memory pool used for pooled device allocations.
  cudaMemPool_t pool() { return pool_; }

  /// Block the calling CPU thread until all work queued on stream() up to this point has
  /// completed. Because frees and consumer waits share this stream, this also drains them.
  void blockCpuUntilStreamCompletion();

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
