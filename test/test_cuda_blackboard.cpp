#include "cuda_blackboard/cuda_blackboard.hpp"
#include "cuda_test_utils.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>

namespace cuda_blackboard::test
{

// Fixture for CudaBlackboard tests: requires a CUDA device (CudaImage's constructor creates a CUDA
// event) and gathers the per-test producer-name and image factory helpers. Each test uses a unique
// producer name so cases stay independent despite the process-wide singleton blackboard.
class CudaBlackboardTest : public CudaDeviceTest
{
protected:
  static CudaBlackboard<CudaImage> & Blackboard()
  {
    return CudaBlackboard<CudaImage>::getInstance();
  }

  static std::string UniqueProducerName()
  {
    const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    return std::string(test_info->test_suite_name()) + "." + test_info->name();
  }

  static std::unique_ptr<CudaImage> MakeImage(const uint32_t height)
  {
    auto image = std::make_unique<CudaImage>();
    image->height = height;
    return image;
  }
};

TEST_F(CudaBlackboardTest, QueryByInstanceIdAndProducerConsumesTickets)
{
  auto & blackboard = Blackboard();
  const std::string producer = UniqueProducerName();

  const uint64_t instance_id = blackboard.registerData(producer, MakeImage(10), 2);

  const auto by_id = blackboard.queryData(instance_id);
  ASSERT_NE(by_id, nullptr);
  EXPECT_EQ(by_id->height, 10U);

  const auto by_producer = blackboard.queryData(producer);
  ASSERT_NE(by_producer, nullptr);
  EXPECT_EQ(by_producer->height, 10U);

  // After 2 (== # of tickets on `registerData`) times queries,
  // the data should be erased from the blackboard
  EXPECT_EQ(blackboard.queryData(instance_id), nullptr);
  EXPECT_EQ(blackboard.queryData(producer), nullptr);
}

TEST_F(CudaBlackboardTest, DuplicateProducerReplacesPreviousData)
{
  auto & blackboard = Blackboard();
  const std::string producer = UniqueProducerName();

  // registration using the same producer_name replaces older one by newer one
  const uint64_t old_instance_id = blackboard.registerData(producer, MakeImage(1), 2);
  const uint64_t new_instance_id = blackboard.registerData(producer, MakeImage(2), 1);

  // different registration should return different instance ID
  EXPECT_NE(old_instance_id, new_instance_id);
  // Query by older (= replaced) instance ID should fail (return nullptr)
  EXPECT_EQ(blackboard.queryData(old_instance_id), nullptr);

  const auto current = blackboard.queryData(producer);
  ASSERT_NE(current, nullptr);
  EXPECT_EQ(current->height, 2U);

  EXPECT_EQ(blackboard.queryData(new_instance_id), nullptr);
}

TEST_F(CudaBlackboardTest, MissingKeysReturnNullptr)
{
  auto & blackboard = Blackboard();
  const uint64_t removed_instance_id =
    blackboard.registerData(UniqueProducerName() + ".removed", MakeImage(3), 1);

  // only 1 (== # of tickets on `registerData`) query returns valid data
  ASSERT_NE(blackboard.queryData(removed_instance_id), nullptr);

  EXPECT_EQ(blackboard.queryData(removed_instance_id), nullptr);
  EXPECT_EQ(blackboard.queryData(UniqueProducerName()), nullptr);
}

}  // namespace cuda_blackboard::test
