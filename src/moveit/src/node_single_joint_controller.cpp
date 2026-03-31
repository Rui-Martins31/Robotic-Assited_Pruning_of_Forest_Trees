// Note:
// To make this script work, you have to activate the forward_position_controller by using this command:
//      ros2 control switch_controllers --deactivate ur5_arm_controller --activate forward_position_controller

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <chrono>
#include <cmath>

// Constants
const std::string NODE_NAME = "single_joint_commander";

const std::string PUB_TOPIC_NAME = "/forward_position_controller/commands";
const std::string SUB_TOPIC_NAME = "/yolo/position_vector";

// const auto TIMER_DELAY = 20ms;

using namespace std::chrono_literals;
using std::placeholders::_1;

//
class SingleJointCommander : public rclcpp::Node
{
public:
    SingleJointCommander()
    : Node(NODE_NAME)
    {
        // Publisher to the controller's command topic
        command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            PUB_TOPIC_NAME,
            10
        );

        // Subscriber
        yolo_subscriber_ = this->create_subscription<geometry_msgs::msg::Point> (
            SUB_TOPIC_NAME,
            10,
            std::bind(&SingleJointCommander::subscribe_callback, this, _1)
        );

        // Timer to publish at 50 Hz
        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&SingleJointCommander::publish_command, this)
        );

        RCLCPP_INFO(this->get_logger(), "Single joint commander started!");
    }

private:
    void subscribe_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        this->position.x = msg.get()->x;
        this->position.y = msg.get()->y;
        this->position.z = msg.get()->z;

        RCLCPP_INFO(this->get_logger(), "X value: %f", this->position.x);
    }

    void publish_command()
    {
        auto msg = std_msgs::msg::Float64MultiArray();

        // Joint angle
        // TO DO: fetch the current wrist_2_joint position and add/subtract the value from there
        double joint_angle = -0.35*(M_PI * this->position.x) - M_PI/2;
        msg.data = {joint_angle};

        command_publisher_->publish(msg);

        // DEBUG
        RCLCPP_INFO(this->get_logger(), "Command: %.3f rad", joint_angle);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr yolo_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Point position;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleJointCommander>());
    rclcpp::shutdown();
    return 0;
}