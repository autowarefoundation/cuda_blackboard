#include "cuda_blackboard/cuda_unique_ptr.hpp"
#include "cuda_test_utils.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace cuda_blackboard::test
{

using CudaUniquePtrTest = CudaDeviceTest;
using HostUniquePtrTest = CudaDeviceTest;

TEST_F(CudaUniquePtrTest, AllocatesDeviceObject)
{
  auto device_value = make_unique<int>();
  // cuda_blackboard::make_unique is synchronous. So device_value.get() should always hold valid
  // value at this stage
  ASSERT_NE(device_value.get(), nullptr);

  const int input = 42;
  int output = 0;
  EXPECT_TRUE(
    CudaSucceeded(cudaMemcpy(device_value.get(), &input, sizeof(input), cudaMemcpyHostToDevice)));
  EXPECT_TRUE(
    CudaSucceeded(cudaMemcpy(&output, device_value.get(), sizeof(output), cudaMemcpyDeviceToHost)));
  // No synchronization is required due to synchronous API (cudaMemcpy)
  EXPECT_EQ(output, input);
}

TEST_F(CudaUniquePtrTest, AllocatesDeviceArray)
{
  constexpr std::size_t array_size = 4;
  auto device_array = make_unique<uint8_t[]>(array_size);
  // cuda_blackboard::make_unique is synchronous. So device_value.get() should always hold valid
  // value at this stage
  ASSERT_NE(device_array.get(), nullptr);

  const std::array<uint8_t, array_size> input{{1, 2, 3, 4}};
  std::array<uint8_t, array_size> output{{}};
  EXPECT_TRUE(CudaSucceeded(cudaMemcpy(
    device_array.get(), input.data(), input.size() * sizeof(uint8_t), cudaMemcpyHostToDevice)));
  EXPECT_TRUE(CudaSucceeded(cudaMemcpy(
    output.data(), device_array.get(), output.size() * sizeof(uint8_t), cudaMemcpyDeviceToHost)));
  // No synchronization is required due to synchronous API (cudaMemcpy)
  EXPECT_EQ(output, input);
}

TEST_F(HostUniquePtrTest, AllocatesPinnedHostObject)
{
  auto host_value = make_host_unique<int>();
  ASSERT_NE(host_value.get(), nullptr);

  *host_value = 7;
  EXPECT_EQ(*host_value, 7);
}

TEST_F(HostUniquePtrTest, AllocatesPinnedHostArray)
{
  constexpr std::size_t array_size = 4;
  auto host_array = make_host_unique<int[]>(array_size);
  ASSERT_NE(host_array.get(), nullptr);

  for (std::size_t i = 0; i < array_size; ++i) {
    host_array[i] = static_cast<int>(i);
  }

  for (std::size_t i = 0; i < array_size; ++i) {
    EXPECT_EQ(host_array[i], static_cast<int>(i));
  }
}

}  // namespace cuda_blackboard::test
