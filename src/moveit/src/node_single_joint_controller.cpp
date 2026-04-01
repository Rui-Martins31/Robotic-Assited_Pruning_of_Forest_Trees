// Note:
// To make this script work, you have to activate the forward_position_controller by using this command:
//      ros2 control switch_controllers --deactivate ur5_arm_controller --activate forward_position_controller

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cmath>

// Constants
static const std::string NODE_NAME = "single_joint_commander";

static const std::string PUB_TOPIC_NAME = "/forward_position_controller/commands";
static const std::string SUB_TOPIC_NAME_YOLO = "/yolo/position_vector";
static const std::string SUB_TOPIC_NAME_JOINT_STATE = "/joint_states";

static const std::string JOINT_NAME = "wrist_2_joint";

// const auto TIMER_DELAY = 20ms;

// Controller
static const float KP = 0.55;
static const float KI = 0.0;
static const float KD = 0.0;

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
            SUB_TOPIC_NAME_YOLO,
            10,
            std::bind(&SingleJointCommander::subscribe_yolo_callback, this, _1)
        );
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState> (
            SUB_TOPIC_NAME_JOINT_STATE,
            10,
            std::bind(&SingleJointCommander::subscribe_joint_state_callback, this, _1)
        );

        // Timer to publish at 50 Hz
        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&SingleJointCommander::publish_command, this)
        );

        RCLCPP_INFO(this->get_logger(), "Single joint commander started!");
    }

private:
    void subscribe_yolo_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        this->position.x = msg.get()->x;
        this->position.y = msg.get()->y;
        this->position.z = msg.get()->z;

        RCLCPP_INFO(this->get_logger(), "X value: %f", this->position.x);
    }
    void subscribe_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Get joint index
        int joint_idx = 0;
        for ( ; msg.get()->name.size(); joint_idx++) {
            if (msg.get()->name[joint_idx] == JOINT_NAME) {
                // std::cout << "Joint name: " + msg.get()->name[joint_idx] << std::endl;
                break;
            }
        }

        // Get current joint position
        this->joint_position_current = msg.get()->position[joint_idx];

        RCLCPP_INFO(this->get_logger(), "Current joint position: %.3f rad", this->joint_position_current);
    }


    void publish_command()
    {
        auto msg = std_msgs::msg::Float64MultiArray();

        // Joint angle
        // TO DO: fetch the current wrist_2_joint position and add/subtract the value from there
        // double joint_angle = -KP*(M_PI * this->position.x) - M_PI/2;
        double joint_angle = this->joint_position_current - KP * this->position.x;
        msg.data = {joint_angle};

        command_publisher_->publish(msg);

        // Reset
        this->position.x = 0.0;

        // DEBUG
        RCLCPP_INFO(this->get_logger(), "Command: %.3f rad", joint_angle);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr yolo_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Point position;
    double joint_position_current;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleJointCommander>());
    rclcpp::shutdown();
    return 0;
}