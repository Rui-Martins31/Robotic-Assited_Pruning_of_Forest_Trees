// More information:
// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp

#include <chrono>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

// Constants
static const std::string NODE_NAME = "kinematic_controller";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

static const std::string PLANNING_GROUP = "ur5_arm";
static const bool USE_SIM_TIME = true;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({rclcpp::Parameter("use_sim_time", USE_SIM_TIME)});
    auto move_group_node = rclcpp::Node::make_shared(NODE_NAME, node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    // MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // Class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Get current state
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10.0);
    if (!current_state) {
        RCLCPP_ERROR(LOGGER, "Failed to get current robot state. Is use_sim_time set correctly?");
        rclcpp::shutdown();
        executor_thread.join();
        return 1;
    }
    // const moveit::core::JointModelGroup* joint_model_group =
    //     current_state->getJointModelGroup(PLANNING_GROUP);

    // DEBUG
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout << std::endl;

    /*--- Plannning ---*/
    RCLCPP_INFO(LOGGER, "Planning pose 1...");
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x    = 0.28;
    target_pose1.position.y    = -0.2;
    target_pose1.position.z    = 0.5;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    /*--- Executing ---*/
    if (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "Executed plan 1 (pose goal) successfuly");
    } else {
        RCLCPP_INFO(LOGGER, "Failed to executed plan 1");
    }

    // Wait for Ctrl+C to shutdown
    // while (rclcpp::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    executor.cancel();
    executor_thread.join();

    return 0;
}