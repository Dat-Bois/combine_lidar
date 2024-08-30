#include "combine_lidar/combine_livox.hpp"
#include "combine_lidar/math_utils.hpp"
#include <chrono>
#include <functional>
#include <thread>
#include <csignal>

using namespace std::chrono_literals;

extern std::string left_cloud_topic;
extern std::string right_cloud_topic;
extern std::string combined_cloud_topic;
extern double x_distance_to_center;
extern double y_distance_to_center;
extern double angle_c;

extern PointCloud2Ptr left_cloud;
extern PointCloud2Ptr right_cloud;

extern MatrixXd left_xyz;
extern MatrixXd right_xyz;

std::mutex left_cloud_mutex;
std::mutex right_cloud_mutex;

std::string left_cloud_topic = "/livox/lidar_3WEDH7600103311";
std::string right_cloud_topic = "/livox/lidar_3WEDH7600104801"; // /livox/lidar_3WEDH7600104801
std::string combined_cloud_topic = "/livox/combined_lidar";

double x_distance_to_center = 0.2; // meters
double y_distance_to_center = 0; // meters 
double angle_c = 30; // degrees, clockwise rotation

PointCloud2Ptr left_cloud;
PointCloud2Ptr right_cloud;

MatrixXd left_xyz;
MatrixXd right_xyz;

void SigHandle(int sig) 
{
  if (sig == SIGINT) {} // to ignore warning
  RCLCPP_INFO(rclcpp::get_logger("combine_livox"), "Shutting down combine_livox node...");
  rclcpp::shutdown();
}

PointCloud2Ptr combine_lidar() {
  std::lock_guard<std::mutex> left_lock(left_cloud_mutex, std::adopt_lock);
  std::lock_guard<std::mutex> right_lock(right_cloud_mutex, std::adopt_lock);

  if (left_cloud == nullptr || right_cloud == nullptr) {
    return nullptr;
  }

  PointCloud2Ptr combined_cloud = std::make_shared<PointCloud2>(*left_cloud);
  combined_cloud->header.stamp = rclcpp::Clock().now();
  combined_cloud->width = left_cloud->width + right_cloud->width;
  combined_cloud->height = 1;
  combined_cloud->data.resize(combined_cloud->width * combined_cloud->height * combined_cloud->point_step);

  MatrixXd combined_xyz(left_xyz.rows() + right_xyz.rows(), 3);
  combined_xyz << left_xyz, right_xyz;

  update_cloud(combined_cloud, combined_xyz);

  return combined_cloud;
}

void right_PointCloud2Callback(const PointCloud2Ptr msg) {
  MatrixXd xyz = create_cloud(msg);
  MatrixXd rotated_xyz = rotatePointCloud(xyz, angle_c, x_distance_to_center, y_distance_to_center, 0.0);
  update_cloud(msg, rotated_xyz);
  std::lock_guard<std::mutex> lock(right_cloud_mutex);
  right_cloud = msg;
  right_xyz = rotated_xyz;
}

void left_PointCloud2Callback(const PointCloud2Ptr msg) {
  MatrixXd xyz = create_cloud(msg);
  MatrixXd rotated_xyz = rotatePointCloud(xyz, -angle_c, -x_distance_to_center, y_distance_to_center, 0.0);
  update_cloud(msg, rotated_xyz);
  std::lock_guard<std::mutex> lock(left_cloud_mutex);
  left_cloud = msg;
  left_xyz = rotated_xyz;
}

CombineLivox::CombineLivox()
: Node("combine_livox")
{
  left_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  right_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto leftopts = rclcpp::SubscriptionOptions();
  leftopts.callback_group = left_group;
  auto rightopts = rclcpp::SubscriptionOptions();
  rightopts.callback_group = right_group;

  left_cloud_sub_ = this->create_subscription<PointCloud2>(left_cloud_topic, 10, left_PointCloud2Callback, leftopts);
  right_cloud_sub_ = this->create_subscription<PointCloud2>(right_cloud_topic, 10, right_PointCloud2Callback, rightopts);
  combined_cloud_pub_ = this->create_publisher<PointCloud2>(combined_cloud_topic, 10);
  comb_lidar_status_pub_ = this->create_publisher<std_msgs::msg::String>("comb_lidar_status", 10);

  timer_ = this->create_wall_timer(30ms, std::bind(&CombineLivox::timer_callback, this));
}

void CombineLivox::timer_callback() {
  std_msgs::msg::String msg;
  PointCloud2Ptr combined_cloud = combine_lidar();
  if (combined_cloud != nullptr) {
    this->combined_cloud_pub_->publish(*combined_cloud);
    msg.data = "Combined cloud published";
  } else {
    msg.data = "Combined cloud not published";
  }
  this->comb_lidar_status_pub_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<CombineLivox>();
  std::signal(SIGINT, SigHandle);
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(),"Starting the combine_livox node...");
  executor.spin();
  RCLCPP_INFO(node->get_logger(), "Shutting down combine_livox node...");
  rclcpp::shutdown();
  return 0;
}
