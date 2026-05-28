// Copyright 2024 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This code is licensed under CC0 1.0 Universal (Public Domain).
// You can use this without any limitation.
// https://creativecommons.org/publicdomain/zero/1.0/deed.en
// borrowed from https://proc-cpuinfo.fixstars.com/2019/02/cuda_smart_pointer/

#ifndef CUDA_BLACKBOARD__CUDA_UNIQUE_PTR_HPP_
#define CUDA_BLACKBOARD__CUDA_UNIQUE_PTR_HPP_

#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#include <memory>
#include <sstream>
#include <stdexcept>
#include <type_traits>

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

// Deleter for device memory. By default it frees synchronously with cudaFree
// (which implicitly synchronizes the whole device). When constructed with a
// stream it instead frees with cudaFreeAsync on that stream, so the free is
// ordered on the stream rather than stalling the device.
//
// The deleter keeps the same type regardless of mode, so CudaUniquePtr<T> (and
// therefore every member that uses it, e.g. CudaPointCloud2::data) is unchanged
// and existing code remains source-compatible.
//
// The free stream can be re-targeted after construction with set_stream(). This
// is what lets a buffer allocated on the producer stream be freed on a
// consumer's stream instead (ordering the free after that consumer's reads).
// set_stream() is const and the stream is mutable on purpose: the free stream
// is resource-reclamation plumbing, not part of the buffer's value, so it is
// settable even through a unique_ptr owned by a shared_ptr<const T>.
struct CudaDeleter
{
  CudaDeleter() = default;
  explicit CudaDeleter(cudaStream_t stream) : stream_(stream), async_(true) {}

  void operator()(void * p) const
  {
    if (async_) {
      CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaFreeAsync(p, stream_));
    } else {
      CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaFree(p));
    }
  }

  // Re-target the stream the buffer will be freed on. No effect for a
  // synchronously allocated buffer (one not created via the stream-aware
  // make_unique), which must keep its cudaFree path.
  void set_stream(cudaStream_t stream) const
  {
    if (async_) {
      stream_ = stream;
    }
  }

  mutable cudaStream_t stream_{nullptr};
  bool async_{false};
};
template <typename T>
using CudaUniquePtr = std::unique_ptr<T, CudaDeleter>;

template <typename T>
typename std::enable_if_t<std::is_array<T>::value, CudaUniquePtr<T>> make_unique(
  const std::size_t n)
{
  using U = typename std::remove_extent_t<T>;
  U * p;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(U) * n));
  return CudaUniquePtr<T>{p};
}

template <typename T>
CudaUniquePtr<T> make_unique()
{
  T * p;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaMalloc(reinterpret_cast<void **>(&p), sizeof(T)));
  return CudaUniquePtr<T>{p};
}

// Stream-ordered allocations: the memory is allocated with cudaMallocAsync on
// the given stream and the returned pointer carries a deleter that frees it
// with cudaFreeAsync on the same stream. Requires CUDA >= 11.2 with memory-pool
// support (cudaDevAttrMemoryPoolsSupported).
template <typename T>
typename std::enable_if_t<std::is_array<T>::value, CudaUniquePtr<T>> make_unique(
  const std::size_t n, cudaStream_t stream)
{
  using U = typename std::remove_extent_t<T>;
  U * p;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    ::cudaMallocAsync(reinterpret_cast<void **>(&p), sizeof(U) * n, stream));
  return CudaUniquePtr<T>{p, CudaDeleter{stream}};
}

template <typename T>
typename std::enable_if_t<!std::is_array<T>::value, CudaUniquePtr<T>> make_unique(
  cudaStream_t stream)
{
  T * p;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(
    ::cudaMallocAsync(reinterpret_cast<void **>(&p), sizeof(T), stream));
  return CudaUniquePtr<T>{p, CudaDeleter{stream}};
}

struct HostDeleter
{
  void operator()(void * p) const { CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaFreeHost(p)); }
};
template <typename T>
using HostUniquePtr = std::unique_ptr<T, HostDeleter>;

template <typename T>
typename std::enable_if_t<std::is_array<T>::value, HostUniquePtr<T>> make_host_unique(
  const std::size_t n)
{
  using U = typename std::remove_extent_t<T>;
  U * p;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaMallocHost(reinterpret_cast<void **>(&p), sizeof(U) * n));
  return HostUniquePtr<T>{p};
}

template <typename T>
HostUniquePtr<T> make_host_unique()
{
  T * p;
  CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaMallocHost(reinterpret_cast<void **>(&p), sizeof(T)));
  return HostUniquePtr<T>{p};
}

}  // namespace cuda_blackboard

#endif  // CUDA_BLACKBOARD__CUDA_UNIQUE_PTR_HPP_
