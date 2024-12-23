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

struct CudaDeleter
{
  void operator()(void * p) const { CUDA_BLACKBOARD_CHECK_CUDA_ERROR(::cudaFree(p)); }
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
