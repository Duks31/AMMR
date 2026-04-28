#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cika_manipulator/action/robotarm_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <string>

using namespace std::placeholders;

namespace cika_manipulator
{ // ADDED: Missing opening brace for namespace

class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Lean AMMR Task Server");
    
    // Back to the simple, default Action Server creation
    action_server_ = rclcpp_action::create_server<cika_manipulator::action::RobotarmTask>(
        this, "task_server", 
        std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<cika_manipulator::action::RobotarmTask>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
  std::string arm_target_pose_, gripper_target_pose_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const cika_manipulator::action::RobotarmTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request for Task Number: %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<cika_manipulator::action::RobotarmTask>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "EMERGENCY STOP: Canceling goal");
    if(arm_move_group_) arm_move_group_->stop();
    if(gripper_move_group_) gripper_move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<cika_manipulator::action::RobotarmTask>> goal_handle)
  {
    // The single detached thread: Keeps MoveIt listening to Gazebo physics
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<cika_manipulator::action::RobotarmTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal...");
    auto result = std::make_shared<cika_manipulator::action::RobotarmTask::Result>();
    auto goal = goal_handle->get_goal();

    bool is_first_run = false;

    if(!arm_move_group_){
      arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
      
      // CLAUDE'S OPEN-LOOP FIX: Force MoveIt to plan paths that take 10x longer, 
      // matching RViz and giving Gazebo physics time to actually arrive!
      arm_move_group_->setMaxVelocityScalingFactor(0.1);
      arm_move_group_->setMaxAccelerationScalingFactor(0.1);
      
      is_first_run = true;
    }
    if(!gripper_move_group_){
      gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
      gripper_move_group_->setMaxVelocityScalingFactor(0.1);
      gripper_move_group_->setMaxAccelerationScalingFactor(0.1);
      is_first_run = true;
    }

    if (is_first_run) {
      RCLCPP_INFO(get_logger(), "Waking up MoveIt... Waiting 2 seconds for TF tree and Joint States.");
      rclcpp::sleep_for(std::chrono::seconds(2));
    }

    bool is_pose_target = false;

    // --- TASK LOGIC ---
    if (goal->task_number == 0) {
      arm_target_pose_ = "home";
      gripper_target_pose_ = "open";
    } 
    else if (goal->task_number == 1) {
      arm_target_pose_ = "drop_1";
      gripper_target_pose_ = "open"; 
    } 
    else if (goal->task_number == 2) {
      arm_target_pose_ = "drop_2";
      gripper_target_pose_ = "open"; 
    } 
    else if (goal->task_number == 3) {
      is_pose_target = true;
      gripper_target_pose_ = "closed"; 
    } 
    else if (goal->task_number == 4) {
      arm_target_pose_ = "rest";
      gripper_target_pose_ = "closed"; 
    } 
    else {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number provided!");
      return;
    }

    // 1. Setup and Move the ARM first
    if (is_pose_target) {
      arm_move_group_->setPositionTarget(goal->x, goal->y, goal->z);
    } else {
      arm_move_group_->setNamedTarget(arm_target_pose_);
    }
    
    RCLCPP_INFO(get_logger(), "Moving Arm...");
    
    // .move() handles planning AND execution sequentially without forcing a rigid start state
    bool arm_success = (arm_move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    
    // 2. ONLY if the arm succeeds, move the GRIPPER
    if(arm_success) {
      RCLCPP_INFO(get_logger(), "Arm reached target. Moving Gripper...");
      
      // Give the gripper slightly more tolerance so Gazebo physics don't cause an instant abort
      gripper_move_group_->setGoalJointTolerance(0.01); 
      gripper_move_group_->setNamedTarget(gripper_target_pose_);
      
      bool gripper_success = (gripper_move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (gripper_success) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Task completed successfully.");
        return;
      } else {
        RCLCPP_ERROR(get_logger(), "Gripper failed to actuate.");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Arm path planning failed. Is the object out of reach?");
    }
  
    // If we reach here, something failed
    result->success = false;
    goal_handle->abort(result);
  } // FIXED: Removed the extra semicolon here
};

}  // namespace cika_manipulator

// Back to the Simple, Single-Threaded Engine
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", true}});
  
  auto node = std::make_shared<cika_manipulator::TaskServer>(options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} // FIXED: Removed the extra semicolon here