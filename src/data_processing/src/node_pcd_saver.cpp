#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;

// Constants
#define NODE_NAME "pcd_saver_node"

#define PATH_SAVE "./output/pcd_processing/pcds"

#define CAMERA_TOPIC_PCD "camera/points"
#define TIMER_DELAY  0.25 // seconds

// Class
class PcdSaver : public rclcpp::Node
{
    public:
        PcdSaver() : Node(NODE_NAME), count_(0)
        {
            // Folder path
            if (fs::exists(PATH_SAVE)) {
                fs::remove_all(PATH_SAVE);
            }
            fs::create_directories(PATH_SAVE);

            // Subscriber
            subscription_pcd = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                CAMERA_TOPIC_PCD,
                10,
                std::bind(&PcdSaver::topic_pcd_callback, this, std::placeholders::_1)
            );

            // Timer
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(TIMER_DELAY),
                std::bind(&PcdSaver::timer_callback, this)
            );

            RCLCPP_INFO(this->get_logger(), "Pcd Saver Node started. Saving to: %s", PATH_SAVE);
        }

    private:
        void topic_pcd_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pcl::fromROSMsg(*msg, pcd_current);
        }
        
        void timer_callback() {
            std::string filename = (std::string)PATH_SAVE + "pcd_" + std::to_string(count_) + ".pcd";
            pcl::io::savePCDFileASCII(filename, pcd_current);

            count_++;
            RCLCPP_INFO(this->get_logger(), "Saved %zu points", pcd_current.size());
        }

        // PointCloud
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pcd;
        pcl::PointCloud<pcl::PointXYZ> pcd_current;
        int count_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdSaver>());
    rclcpp::shutdown();
    return 0;
}