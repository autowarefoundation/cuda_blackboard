#include "cuda_blackboard/cuda_blackboard.hpp"
#include "cuda_blackboard/cuda_image.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"
#include "fixtures.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>

#include <utility>

namespace cuda_blackboard
{

class CudaBlackboardTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    CudaBlackboard<CudaPointCloud2>::getInstance().reset();
    CudaBlackboard<CudaImage>::getInstance().reset();
  }
};

TEST_F(CudaBlackboardTest, SingletonBehavior)
{
  auto & blackboard1 = CudaBlackboard<CudaPointCloud2>::getInstance();
  auto & blackboard2 = CudaBlackboard<CudaPointCloud2>::getInstance();

  // Should be the same instance
  EXPECT_EQ(&blackboard1, &blackboard2);
}

TEST_F(CudaBlackboardTest, RegisterAndQueryData)
{
  auto & blackboard = CudaBlackboard<CudaPointCloud2>::getInstance();

  auto test_pc = create_test_point_cloud();
  const std::string producer_name = "test_producer";
  const size_t num_tickets = 2;

  // Store reference data for comparison
  auto expected_frame_id = test_pc->header.frame_id;
  auto expected_width = test_pc->width;
  auto expected_height = test_pc->height;

  // Register data
  uint64_t instance_id = blackboard.registerData(producer_name, std::move(test_pc), num_tickets);
  EXPECT_NE(instance_id, 0);

  // Query by producer name
  auto retrieved_data_by_name = blackboard.queryData(producer_name);
  ASSERT_NE(retrieved_data_by_name, nullptr);
  EXPECT_EQ(retrieved_data_by_name->header.frame_id, expected_frame_id);
  EXPECT_EQ(retrieved_data_by_name->width, expected_width);
  EXPECT_EQ(retrieved_data_by_name->height, expected_height);

  // Query by instance ID
  auto retrieved_data_by_id = blackboard.queryData(instance_id);
  ASSERT_NE(retrieved_data_by_id, nullptr);
  EXPECT_EQ(retrieved_data_by_id->header.frame_id, expected_frame_id);

  // Should be the same object
  EXPECT_EQ(retrieved_data_by_name.get(), retrieved_data_by_id.get());
}

TEST_F(CudaBlackboardTest, QueryNonexistentData)
{
  auto & blackboard = CudaBlackboard<CudaPointCloud2>::getInstance();

  // Query non-existent producer
  auto result = blackboard.queryData("nonexistent_producer");
  EXPECT_EQ(result, nullptr);

  // Query non-existent instance ID
  auto result_by_id = blackboard.queryData(999999);
  EXPECT_EQ(result_by_id, nullptr);
}

TEST_F(CudaBlackboardTest, OverwriteProducerData)
{
  auto & blackboard = CudaBlackboard<CudaPointCloud2>::getInstance();

  const std::string producer_name = "overwrite_producer";

  // Register first data
  auto first_pc = create_test_point_cloud();
  first_pc->header.frame_id = "first_frame";
  auto expected_first_frame = first_pc->header.frame_id;
  uint64_t first_id = blackboard.registerData(producer_name, std::move(first_pc), 1);

  // Register second data with same producer name
  auto second_pc = create_test_point_cloud();
  second_pc->header.frame_id = "second_frame";
  auto expected_second_frame = second_pc->header.frame_id;
  uint64_t second_id = blackboard.registerData(producer_name, std::move(second_pc), 1);

  // IDs should be different
  EXPECT_NE(first_id, second_id);

  // Query should return the latest data
  auto retrieved = blackboard.queryData(producer_name);
  ASSERT_NE(retrieved, nullptr);
  EXPECT_EQ(retrieved->header.frame_id, expected_second_frame);

  // Old instance ID shall have been dropped
  auto old_data = blackboard.queryData(first_id);
  EXPECT_EQ(old_data, nullptr);
}

TEST_F(CudaBlackboardTest, ImageBlackboard)
{
  auto & blackboard = CudaBlackboard<CudaImage>::getInstance();

  auto test_img = create_test_image();
  const std::string producer_name = "image_producer";

  // Store reference data
  auto expected_width = test_img->width;
  auto expected_height = test_img->height;
  auto expected_encoding = test_img->encoding;

  // Register and query
  blackboard.registerData(producer_name, std::move(test_img), 1);
  auto retrieved = blackboard.queryData(producer_name);

  ASSERT_NE(retrieved, nullptr);
  EXPECT_EQ(retrieved->width, expected_width);
  EXPECT_EQ(retrieved->height, expected_height);
  EXPECT_EQ(retrieved->encoding, expected_encoding);
}

TEST_F(CudaBlackboardTest, MultipleTickets)
{
  auto & blackboard = CudaBlackboard<CudaPointCloud2>::getInstance();

  auto test_pc = create_test_point_cloud();
  const std::string producer_name = "multi_ticket_producer";
  const size_t num_tickets = 5;

  // Register data with multiple tickets
  blackboard.registerData(producer_name, std::move(test_pc), num_tickets);

  // Should be able to query multiple times
  for (size_t i = 0; i < num_tickets; ++i) {
    auto retrieved = blackboard.queryData(producer_name);
    EXPECT_NE(retrieved, nullptr) << "Failed to query on attempt " << i;
  }

  auto retrieved = blackboard.queryData(producer_name);
  EXPECT_EQ(retrieved, nullptr) << "Should not be able to query after tickets are exhausted";
}

TEST_F(CudaBlackboardTest, ZeroTickets)
{
  auto & blackboard = CudaBlackboard<CudaPointCloud2>::getInstance();

  auto test_pc = create_test_point_cloud();
  const std::string producer_name = "zero_ticket_producer";

  // Register data with zero tickets (should not be registered)
  auto id = blackboard.registerData(producer_name, std::move(test_pc), 0);

  // Should not be able to query data
  auto retrieved = blackboard.queryData(producer_name);
  EXPECT_EQ(retrieved, nullptr) << "Should not be able to query data with zero tickets";

  retrieved = blackboard.queryData(id);
  EXPECT_EQ(retrieved, nullptr) << "Should not be able to query data with zero tickets";
}

TEST_F(CudaBlackboardTest, OneTicketIsZeroCopy)
{
  auto & blackboard = CudaBlackboard<CudaPointCloud2>::getInstance();

  auto test_pc = create_test_point_cloud();
  const std::string producer_name = "one_ticket_producer";

  auto original_gpu_pointer = test_pc->data.get();

  // Register data with one ticket
  blackboard.registerData(producer_name, std::move(test_pc), 1);

  // Query data
  auto retrieved = blackboard.queryData(producer_name);
  EXPECT_NE(retrieved, nullptr) << "Should be able to query data with one ticket";

  auto retrieved_gpu_pointer = retrieved->data.get();
  EXPECT_EQ(original_gpu_pointer, retrieved_gpu_pointer) << "Data should be zero-copy with one ticket";
}

}  // namespace cuda_blackboard

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
