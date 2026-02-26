#include <rclcpp/rclcpp.hpp>

// Constants
static const std::string NODE_NAME = "pcd_saver";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

int main(/*int argc, char * argv[]*/) {
    RCLCPP_INFO(LOGGER, "Node running...");
    return 0;
}