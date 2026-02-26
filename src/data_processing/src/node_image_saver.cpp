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
#define PATH_SAVE    "./output/image_processing/images"

#define CAMERA_TOPIC_IMAGE "camera/image"
#define CAMERA_TOPIC_DEPTH "camera/depth_image"
#define TIMER_DELAY  1.0 // seconds

// Struct
struct Camera_Content {
    cv_bridge::CvImagePtr image;
    cv_bridge::CvImagePtr depth_image;
};

// Class
class ImageSaver : public rclcpp::Node
{
    public:
        ImageSaver() : Node("image_saver_node"), count_(0)
        {
            // Folder path
            if (fs::exists(PATH_SAVE)) {
                fs::remove_all(PATH_SAVE);
            }
            fs::create_directories(PATH_SAVE);

            // Subscriber
            subscription_image = this->create_subscription<sensor_msgs::msg::Image>(
                CAMERA_TOPIC_IMAGE,
                10,
                std::bind(&ImageSaver::topic_image_callback, this, std::placeholders::_1)
            );
            subscription_depth_image = this->create_subscription<sensor_msgs::msg::Image>(
                CAMERA_TOPIC_DEPTH,
                10,
                std::bind(&ImageSaver::topic_depth_image_callback, this, std::placeholders::_1)
            );

            // Timer
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(TIMER_DELAY),
                std::bind(&ImageSaver::timer_callback, this)
            );

            RCLCPP_INFO(this->get_logger(), "Image Saver Node started. Saving to: %s", PATH_SAVE);
        }

    private:
        void topic_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                // Convert ROS image to OpenCV Mat
                // last_cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                this->camera_cont.image = cv_bridge::toCvCopy(msg, "bgr8");

            } catch (const cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }
        void topic_depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                // Convert ROS image to OpenCV Mat
                this->camera_cont.depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

                // Invert depth values
                double minVal, maxVal;
                cv::minMaxLoc(this->camera_cont.depth_image->image, &minVal, &maxVal);
                this->camera_cont.depth_image->image = maxVal - this->camera_cont.depth_image->image;

            } catch (const cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }

        void timer_callback() {
            if (!this->camera_cont.image || !this->camera_cont.depth_image) {
                return;
            }

            // Construct filename
            std::string filename = (std::string)PATH_SAVE + "/img_" + std::to_string(count_++);

            // Save
            if (cv::imwrite(filename + "_image.png", this->camera_cont.image->image)
                && 
                cv::imwrite(filename + "_depth_image.png", this->camera_cont.depth_image->image)
            ) {
                RCLCPP_INFO(this->get_logger(), "Saved: %s", filename.c_str());
            }
        }

        // Image
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth_image;
        int count_;
        // cv_bridge::CvImagePtr last_cv_ptr;
        Camera_Content camera_cont;

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