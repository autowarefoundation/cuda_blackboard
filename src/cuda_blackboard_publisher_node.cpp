
#include "cuda_blackboard/cuda_adaptation.hpp"
#include "cuda_blackboard/cuda_blackboard_publisher.hpp"
#include "cuda_blackboard/cuda_image.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // for ROS 2 Jazzy or newer
#else
#include <cv_bridge/cv_bridge.h>  // for ROS 2 Humble or older
#endif

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace cuda_blackboard
{

class CudaBlackboardPublisherNode final : public rclcpp::Node
{
public:
  explicit CudaBlackboardPublisherNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("cuda_blackboard_publisher_node", options)
  {
    std::string image_path = this->declare_parameter<std::string>("image_path", "image.png");

    pub_ = std::make_shared<CudaBlackboardPublisher<CudaImage>>(*this, "image_raw");

    RCLCPP_INFO_STREAM(this->get_logger(), "Loading " << image_path);
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Could not read image");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Image size: %d x %d", image.cols, image.rows);

    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = this->now();
    cv_image.header.frame_id = "camera_frame";
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = image;

    cv_image.toImageMsg(ros_image_);

    auto publish_message = [this]() -> void {
      auto cuda_image_ptr = std::make_unique<CudaImage>(ros_image_);
      cuda_image_ptr->header.stamp = this->now();
      RCLCPP_INFO(this->get_logger(), "Publishing message n=%d", count_++);
      pub_->publish(std::move(cuda_image_ptr));
    };

    timer_ = create_wall_timer(std::chrono::seconds(2), publish_message);
  }

private:
  sensor_msgs::msg::Image ros_image_;
  std::shared_ptr<CudaBlackboardPublisher<CudaImage>> pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_{0};
};

}  // namespace cuda_blackboard

RCLCPP_COMPONENTS_REGISTER_NODE(cuda_blackboard::CudaBlackboardPublisherNode)
