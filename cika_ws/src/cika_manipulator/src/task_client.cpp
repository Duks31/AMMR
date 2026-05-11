#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cika_manipulator/action/robotarm_task.hpp"
#include <vector>
#include <chrono>

using RobotarmTask = cika_manipulator::action::RobotarmTask;

struct TaskCommand
{
    int task_num;
    double x, y, z;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create a raw node (No OOP Class needed!)
    auto node = rclcpp::Node::make_shared("autonomous_client");
    auto client = rclcpp_action::create_client<RobotarmTask>(node, "task_server");

    if (!client->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Action server not available! Did you run it?");
        rclcpp::shutdown();
        return 1;
    }

    // --- THE PLAYLIST ---
    std::vector<TaskCommand> sequence = {
        {1, 0.0, 0.0, 0.0},  // Step 1: Pre-pick stance
        {3, 0.2, 0.0, 0.05}, // Step 2: Dynamic Pick
        {4, 0.0, 0.0, 0.0},  // Step 3: Rest state
        {1, 0.0, 0.0, 0.0},  // Step 4: Drop 1
        {4, 0.0, 0.0, 0.0}   // Step 5: Rest
    };

    RCLCPP_INFO(node->get_logger(), "Starting Synchronous AMMR Sequence...");

    // --- THE PROCEDURAL LOOP ---
    for (size_t i = 0; i < sequence.size(); ++i)
    {
        auto current_task = sequence[i];
        RCLCPP_INFO(node->get_logger(), "--- Executing Step %zu: Task %d ---", i + 1, current_task.task_num);

        auto goal_msg = RobotarmTask::Goal();
        goal_msg.task_number = current_task.task_num;
        goal_msg.x = current_task.x;
        goal_msg.y = current_task.y;
        goal_msg.z = current_task.z;

        // 1. Send Goal and BLOCK until the server accepts it
        auto goal_handle_future = client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to send goal. Halting.");
            break;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server. Halting.");
            break;
        }

        // 2. Wait for Execution to physically finish in Gazebo
        auto result_future = client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get result. Halting.");
            break;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node->get_logger(), "Step %zu SUCCEEDED.", i + 1);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Task failed in Gazebo. Halting sequence.");
            break;
        }

        // 3. THE BREATHER (Perfect Terminal Mimic)
        // Because we are not in an async callback, we can safely freeze the thread with sleep_for!
        RCLCPP_INFO(node->get_logger(), "Waiting 2 seconds for Gazebo physics to settle...");
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    RCLCPP_INFO(node->get_logger(), "=== AUTONOMOUS SEQUENCE COMPLETE ===");
    rclcpp::shutdown();
    return 0;
}