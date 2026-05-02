#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

using std::placeholders::_1;

class ArmTrajectoryUnpacker : public rclcpp::Node
{
public:
  ArmTrajectoryUnpacker() : Node("arm_trajectory_unpacker")
  {
    // The single output pipe to the Arduino
    target_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/micro_ros_arm_targets", 10);

    // Subscriber 1: The Main Arm Engine
    arm_state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/arm_controller/state", 10,
      std::bind(&ArmTrajectoryUnpacker::arm_callback, this, _1));

    // Subscriber 2: The Gripper Engine
    gripper_state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/gripper_controller/state", 10,
      std::bind(&ArmTrajectoryUnpacker::gripper_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "C++ Unpacker running. Listening to Arm and Gripper.");
  }

private:
  // We use a helper function so we don't write the exact same packing logic twice
  void publish_simplified_state(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) const
  {
    auto simple_msg = sensor_msgs::msg::JointState();
    
    simple_msg.header = msg->header;
    simple_msg.name = msg->joint_names;
    simple_msg.position = msg->reference.positions;

    target_pub_->publish(simple_msg);
  }

  void arm_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) const
  {
    publish_simplified_state(msg);
  }

  void gripper_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) const
  {
    publish_simplified_state(msg);
  }

  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr arm_state_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr gripper_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTrajectoryUnpacker>());
  rclcpp::shutdown();
  return 0;
}