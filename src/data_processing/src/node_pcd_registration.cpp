// More information:
// https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html

#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

// Helpers
static pcl::PointCloud<pcl::PointXYZ> removeInvalidPoints(const pcl::PointCloud<pcl::PointXYZ>& cloud_in)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    cloud_out.reserve(cloud_in.size());
    for (const auto& pt : cloud_in.points) {
        if (pcl::isFinite(pt)) {
            cloud_out.push_back(pt);
        }
    }
    cloud_out.width = static_cast<uint32_t>(cloud_out.size());
    cloud_out.height = 1;
    cloud_out.is_dense = true;
    return cloud_out;
}

// Namespaces
namespace fs = std::filesystem;

// Constants
#define NODE_NAME "pcd_saver_node"

const std::string PATH_SAVE = "./output/pcd_registration/";

#define CAMERA_TOPIC_PCD "camera/points"
#define TIMER_DELAY  0.25 // seconds

const float ICP_MAX_CORRESPONDENCE_DISTANCE = 0.05;
const float ICP_MAX_ITERATIONS              = 50;
const float ICP_TRANSFORMATION_EPSILON      = 1e-8;
const float ICP_EUCLIDEAN_FITNESS_EPSILON   = 1;


// Class
class PcdSaver : public rclcpp::Node
{
    public:
        PcdSaver() : Node(NODE_NAME)
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

            // ICP
            // Set criteria
            this->icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDENCE_DISTANCE);
            this->icp.setMaximumIterations(ICP_MAX_ITERATIONS);
            this->icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
            this->icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_FITNESS_EPSILON);

            RCLCPP_INFO(this->get_logger(), "Pcd Saver Node started. Saving to: %s", PATH_SAVE.c_str());
        }

    private:
        void topic_pcd_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pcl::fromROSMsg(*msg, pcd_new);
        }
        
        void timer_callback() {
            //
            if (pcd_new.empty()) { return; }
            if (pcd_current.empty()) {
                pcd_current = removeInvalidPoints(pcd_new);
                return;
            }

            // Remove Inf/NaN
            pcl::PointCloud<pcl::PointXYZ> pcd_new_filtered = removeInvalidPoints(pcd_new);

            // Set the input source and target
            icp.setInputSource(pcd_new_filtered.makeShared());
            icp.setInputTarget(pcd_current.makeShared());

            // Perform the alignment
            icp.align(pcd_current);

            // Remove any NaN/Inf introduced by ICP before next iteration
            pcd_current = removeInvalidPoints(pcd_current);
            
            // Obtain the transformation that aligned cloud_source to cloud_source_registered
            // Eigen::Matrix4f transformation = icp.getFinalTransformation();
            RCLCPP_INFO(this->get_logger(), "[func timer_callback] PointCloud is aligned.");
            
            // Save PointCloud
            std::string filename = PATH_SAVE + "pcd_registration" + ".pcd";
            pcl::io::savePCDFileASCII(filename, pcd_current);

            RCLCPP_INFO(this->get_logger(), "[func timer_callback] PointCloud saved.");
        }

        // PointCloud
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pcd;
        pcl::PointCloud<pcl::PointXYZ> pcd_new;
        pcl::PointCloud<pcl::PointXYZ> pcd_current;

        // ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

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