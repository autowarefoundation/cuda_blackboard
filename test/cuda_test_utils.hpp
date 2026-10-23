// Copyright 2026 TIER IV, Inc.
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

#pragma once

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

namespace cuda_blackboard::test
{

inline ::testing::AssertionResult CudaDeviceAvailable()
{
  int device_count = 0;
  const cudaError_t status = cudaGetDeviceCount(&device_count);
  if (status != cudaSuccess) {
    cudaGetLastError();
    return ::testing::AssertionFailure()
           << "CUDA device query failed: " << cudaGetErrorString(status);
  }
  if (device_count == 0) {
    return ::testing::AssertionFailure() << "No CUDA device available";
  }
  return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult CudaSucceeded(const cudaError_t status)
{
  if (status == cudaSuccess) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << cudaGetErrorName(status) << ": " << cudaGetErrorString(status);
}

/// Base fixture for tests that require a CUDA device. Skips the test in SetUp() when no device is
/// available, so derived fixtures and TEST_F bodies can assume a usable device.
class CudaDeviceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto cuda_available = CudaDeviceAvailable();
    if (!cuda_available) {
      GTEST_SKIP() << cuda_available.message();
    }
  }
};

}  // namespace cuda_blackboard::test
