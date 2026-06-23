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
