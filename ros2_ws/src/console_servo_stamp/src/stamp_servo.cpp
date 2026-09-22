#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Humble Servo emits header.stamp=0 (execute now). Preserve DDS publication
// time instead of fabricating freshness at relay receipt. Localhost DDS only.
class StampServo : public rclcpp::Node {
 public:
  StampServo() : Node("console_servo_stamp") {
    publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/console/servo_output_stamped", 1);
    subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/console/servo_output", 1,
      [this](trajectory_msgs::msg::JointTrajectory::UniquePtr msg, const rclcpp::MessageInfo &info) {
        const auto stamp = info.get_rmw_message_info().source_timestamp;
        if (stamp <= 0) return; // adapter watchdog handles missing source time
        msg->header.stamp = rclcpp::Time(stamp, RCL_SYSTEM_TIME);
        publisher_->publish(std::move(msg));
      });
  }
 private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StampServo>());
  rclcpp::shutdown();
  return 0;
}
