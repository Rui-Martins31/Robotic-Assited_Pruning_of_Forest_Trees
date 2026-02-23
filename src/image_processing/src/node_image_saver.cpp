#include <memory>
#include <string>
#include <filesystem>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

// Constants
#define PATH_SAVE    "./output/image_processing/images_raw"

#define CAMERA_TOPIC "camera/image_raw"
#define TIMER_DELAY  1.0 // seconds

// Class
class ImageSaver : public rclcpp::Node
{
    public:
        ImageSaver() : Node("image_saver_node"), count_(0)
        {
            // Folder path
            if (fs::exists(PATH_SAVE)) {
                fs::remove(PATH_SAVE);
            }
            fs::create_directories(PATH_SAVE);

            // Subscriber
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                CAMERA_TOPIC,
                10,
                std::bind(&ImageSaver::topic_callback, this, std::placeholders::_1)
            );

            // Timer
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(TIMER_DELAY),
                std::bind(&ImageSaver::timer_callback, this)
            );

            RCLCPP_INFO(this->get_logger(), "Image Saver Node started. Saving to: %s", PATH_SAVE);
        }

    private:
        void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                // Convert ROS image to OpenCV Mat
                last_cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            } catch (const cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }

        void timer_callback() {
            // Construct filename
            std::string filename = (std::string)PATH_SAVE + "/img_" + std::to_string(count_++) + ".png";

            // Save
            if (cv::imwrite(filename, last_cv_ptr->image)) {
                RCLCPP_INFO(this->get_logger(), "Saved: %s", filename.c_str());
            }
        }

        // Image
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        int count_;
        cv_bridge::CvImagePtr last_cv_ptr;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaver>());
    rclcpp::shutdown();
    return 0;
}