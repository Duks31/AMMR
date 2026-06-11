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
        {4, 0.0, 0.0, 0.0},  // Step 1: Rest
        {0, 0.0, 0.0, 0.0},  // Step 3: Home
        {3, 0.135, 0.282, 0.125}, // Step 2: Dynamic Pick (Example coordinates for a detected object)
        {5, 0.0, 0.0, 0.0},  // Step 3: Pre Drop
        {1, 0.0, 0.0, 0.0},  // Step 4: Drop 1 (Plastic)
        {0, 0.0, 0.0, 0.0}   // Step 5: Rest
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


// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include "cika_manipulator/action/robotarm_task.hpp"
// #include "cika_perception/msg/waste_detection_array.hpp" // Required for the topic
// #include <vector>
// #include <chrono>
// #include <thread>
// #include <atomic>

// using RobotarmTask = cika_manipulator::action::RobotarmTask;
// using WasteDetectionArray = cika_perception::msg::WasteDetectionArray;

// struct TaskCommand
// {
//     int task_num;
//     double x, y, z;
// };

// class AutonomousClient : public rclcpp::Node
// {
// public:
//     AutonomousClient() : Node("autonomous_client"), is_executing_sequence_(false)
//     {
//         // 1. Initialize Action Client
//         action_client_ = rclcpp_action::create_client<RobotarmTask>(this, "task_server");

//         // 2. Initialize Subscriber to the camera's perception topic
//         detection_sub_ = this->create_subscription<WasteDetectionArray>(
//             "/cika/perception/waste_detections", 10,
//             std::bind(&AutonomousClient::detection_callback, this, std::placeholders::_1));

//         RCLCPP_INFO(this->get_logger(), "Autonomous Client Ready. Waiting for camera detections...");
//     }

// private:
//     rclcpp_action::Client<RobotarmTask>::SharedPtr action_client_;
//     rclcpp::Subscription<WasteDetectionArray>::SharedPtr detection_sub_;
    
//     // Atomic flag to prevent triggering a new sequence while the arm is already moving
//     std::atomic<bool> is_executing_sequence_;

//     // --- TOPIC CALLBACK ---
//     void detection_callback(const WasteDetectionArray::SharedPtr msg)
//     {
//         // If the arm is currently busy, ignore new camera frames
//         if (is_executing_sequence_) {
//             return; 
//         }

//         if (msg->detections.empty()) {
//             return;
//         }

//         // Look through the detections for a valid 3D object
//         for (const auto& det : msg->detections) {
//             if (det.has_3d_position) {
                
//                 // Lock the node so it doesn't process any more detections
//                 is_executing_sequence_ = true;

//                 double x = det.position.x;
//                 double y = det.position.y;
//                 double z = det.position.z;
                
//                 RCLCPP_INFO(this->get_logger(), "Valid Target Found! Label: %s at [X:%.2f, Y:%.2f, Z:%.2f]", 
//                             det.label.c_str(), x, y, z);

//                 // Dynamically assign the drop-off zone based on the class label
//                 int drop_task = 0;
//                 if (det.label == "plastic") {
//                     drop_task = 1; // Drop 1
//                 } else if (det.label == "paper") {
//                     drop_task = 2; // Drop 2
//                 } else {
//                     RCLCPP_WARN(this->get_logger(), "Unknown label. Defaulting to Rest.");
//                     drop_task = 4;
//                 }

//                 // Launch a detached worker thread to run the movement sequence.
//                 // This allows the main thread to continue spinning and processing ROS network traffic.
//                 std::thread(&AutonomousClient::execute_sequence, this, x, y, z, drop_task).detach();
                
//                 return; // Exit after finding the first valid object
//             }
//         }
//     }

//     // --- WORKER THREAD FOR SEQUENCE ---
//     void execute_sequence(double target_x, double target_y, double target_z, int drop_task)
//     {
//         if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
//             RCLCPP_ERROR(this->get_logger(), "Action server not available! Aborting sequence.");
//             is_executing_sequence_ = false;
//             return;
//         }

//         // Build the dynamic playlist
//         std::vector<TaskCommand> sequence = {
//             {4, 0.0, 0.0, 0.0},                 // Step 1: Rest (Standby)
//             {3, target_x, target_y, target_z},  // Step 2: Dynamic Pick (Using coordinates)
//             {0, 0.0, 0.0, 0.0},                 // Step 3: Home State
//             {drop_task, 0.0, 0.0, 0.0},         // Step 4: Drop 1 or Drop 2 (Based on Label)
//             {4, 0.0, 0.0, 0.0}                  // Step 5: Rest
//         };

//         RCLCPP_INFO(this->get_logger(), "=== STARTING AMMR SEQUENCE ===");

//         for (size_t i = 0; i < sequence.size(); ++i) {
//             auto current_task = sequence[i];
//             RCLCPP_INFO(this->get_logger(), "--- Executing Step %zu: Task %d ---", i + 1, current_task.task_num);

//             auto goal_msg = RobotarmTask::Goal();
//             goal_msg.task_number = current_task.task_num;
//             goal_msg.x = current_task.x;
//             goal_msg.y = current_task.y;
//             goal_msg.z = current_task.z;

//             // 1. Send Goal
//             auto goal_handle_future = action_client_->async_send_goal(goal_msg);
            
//             // Wait for the server to accept the goal. 
//             // We use .wait() instead of spin_until_future_complete because we are in a background thread.
//             if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
//                 RCLCPP_ERROR(this->get_logger(), "Server took too long to accept goal. Halting.");
//                 break;
//             }

//             auto goal_handle = goal_handle_future.get();
//             if (!goal_handle) {
//                 RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server. Halting.");
//                 break;
//             }

//             // 2. Wait for physical execution
//             auto result_future = action_client_->async_get_result(goal_handle);
//             result_future.wait(); // Block thread until Gazebo/MoveIt finishes

//             auto result = result_future.get();
//             if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//                 RCLCPP_INFO(this->get_logger(), "Step %zu SUCCEEDED.", i + 1);
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "Task failed in Gazebo. Halting sequence.");
//                 break;
//             }

//             // 3. Pause for Gazebo Physics
//             RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for physics to settle...");
//             std::this_thread::sleep_for(std::chrono::seconds(2));
//         }

//         RCLCPP_INFO(this->get_logger(), "=== SEQUENCE COMPLETE. Ready for next object. ===");
        
//         // Unlock the node to accept new camera detections
//         is_executing_sequence_ = false;
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<AutonomousClient>();
    
//     // The main thread spins here indefinitely, handling callbacks, 
//     // while the worker thread handles the blocking sequence.
//     rclcpp::spin(node); 
    
//     rclcpp::shutdown();
//     return 0;
// }

