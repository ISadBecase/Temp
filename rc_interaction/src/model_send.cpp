#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class PosePublisherNode : public rclcpp::Node {
public:
  PosePublisherNode() : Node("pose_publisher_node") {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_topic_test", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&PosePublisherNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = -1.0;  // 你可以改成任何测试坐标
    msg.pose.position.y = 2.0;
    msg.pose.position.z = 0.0;

    // 设置四元数，保持无旋转
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published test PoseStamped");
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
