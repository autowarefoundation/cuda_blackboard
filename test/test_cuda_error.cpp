#include "cuda_blackboard/cuda_error.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace cuda_blackboard::test
{

TEST(CudaError, SuccessDoesNotThrow)
{
  EXPECT_NO_THROW(cuda_check_error(cudaSuccess, __FILE__, __LINE__));
}

TEST(CudaError, FailureThrowsRuntimeError)
{
  EXPECT_THROW(cuda_check_error(cudaErrorMemoryAllocation, __FILE__, __LINE__), std::runtime_error);
}

TEST(CudaError, FailureMessageContainsErrorNameAndLocation)
{
  try {
    cuda_check_error(cudaErrorInvalidValue, "my_file.cpp", 123);
    FAIL() << "Expected std::runtime_error to be thrown";
  } catch (const std::runtime_error & e) {
    const std::string message = e.what();
    EXPECT_NE(message.find(cudaGetErrorName(cudaErrorInvalidValue)), std::string::npos);
    EXPECT_NE(message.find("my_file.cpp"), std::string::npos);
    EXPECT_NE(message.find("123"), std::string::npos);
  }
}

TEST(CudaError, MacroForwardsFileAndLine)
{
  EXPECT_NO_THROW(CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaSuccess));
  EXPECT_THROW(CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaErrorMemoryAllocation), std::runtime_error);
}

}  // namespace cuda_blackboard::test
