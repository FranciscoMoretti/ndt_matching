#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"
#include "ndt_matching/ndt_lib.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/convert.h>

void print_usage()
{
  printf("Usage for listener app:\n");
  printf("listener [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to subscribe. Defaults to "
    "chatter.\n");
}

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  explicit Listener(
    const std::string & topic_name,
    const std::string & topic_name2 = "map")
  : Node("listener")
  {

    ndt_matching_localizer = ndt_matching::NdtLib();

    // Temporarily hardcoded initialization
    Eigen::AngleAxisf init_rotation(4.9, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(2, -50, 0);
    ndt_matching_localizer.set_initial_estimation(init_rotation, init_translation);
    // Create a subscription to the topic which can be matched with one or more
    // compatible ROS publishers. Note that not all publishers on the same topic
    // with the same type will be compatible: they must have compatible Quality
    // of Service policies.
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name, 10,
      std::bind(&Listener::scan_callback, this, std::placeholders::_1));
    sub2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name2, 1,
      std::bind(&Listener::map_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_estimation", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  ndt_matching::NdtLib ndt_matching_localizer;
  bool map_loaded = false;

  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // TODO: here you get your map point cloud (one time only)
    // Ensure the map is only loaded once
    if (map_loaded) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'",
      msg->header.frame_id.c_str());
    pcl::PointCloud<pcl::PointXYZ>::Ptr plc_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*plc_point_cloud);
    // Initialize the map
    ndt_matching_localizer.point_cloud_map_callback(plc_point_cloud);
    map_loaded = true;
  }

  void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for
    // zero-copy transport.
    
    // Wait until the map is loaded before processing a scan
    if(!map_loaded){
      return;
    }
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'",
    msg->header.frame_id.c_str());
    // TODO:
    // here you call NdtLib function and pass in the msg as input
    // return a pose message and publish it as
    // https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseStamped.msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr plc_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *plc_point_cloud);
    // Register a scan
    Eigen::Affine3d eigen_pose = 
      ndt_matching_localizer.point_cloud_scan_callback(plc_point_cloud).cast<double>();
    

    geometry_msgs::msg::Pose pose_message = tf2::toMsg(eigen_pose);
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = "map";
    message.header.stamp = msg->header.stamp;
    message.pose = pose_message;
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.header.frame_id.c_str());
    publisher_->publish(message);
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "(x,y,z)(x,y,z,w):" << std::endl;
    std::cout << "(" << pose_message.position.x << ", " << pose_message.position.y << ", "
              << pose_message.position.z << ")(" << pose_message.orientation.x
              << ", " << pose_message.orientation.y << ", " << pose_message.orientation.z << ", "
              <<  pose_message.orientation.z << ")" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    }
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client
  // library. You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("points_raw");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic = std::string(cli_option);
  }

  // Create a node.
  auto node = std::make_shared<Listener>(topic);

  // spin will block until work comes in, execute work as it becomes available,
  // and keep blocking. It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
