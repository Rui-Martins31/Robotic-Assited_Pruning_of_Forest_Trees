#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <cmath>

// Constants
const std::string NODE_NAME = "single_joint_commander";

using namespace std::chrono_literals;

class SingleJointCommander : public rclcpp::Node
{
public:
    SingleJointCommander()
    : Node(NODE_NAME),
      amplitude_(1.0),   // radians (≈57°)
      frequency_(0.5),   // Hz
      phase_(0.0)
    {
        // Publisher to the controller's command topic
        command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands",
            10
        );

        // Timer to publish at 50 Hz
        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&SingleJointCommander::publish_command, this)
        );

        RCLCPP_INFO(this->get_logger(), "Single joint commander started!");
    }

private:
    void publish_command()
    {
        auto msg = std_msgs::msg::Float64MultiArray();

        // One joint → array with a single value
        double position = amplitude_ * std::sin(2.0 * M_PI * frequency_ * phase_);
        msg.data = {position};

        command_publisher_->publish(msg);

        // DEBUG
        // RCLCPP_INFO(this->get_logger(), "Command: %.3f rad", position);

        phase_ += 0.02;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double amplitude_;
    double frequency_;
    double phase_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleJointCommander>());
    rclcpp::shutdown();
    return 0;
}