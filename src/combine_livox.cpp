#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <deque>
#include <mutex>
#include <thread>
#include <typeinfo>

#include <csignal>
#include <condition_variable>

#include <cmath>
#include <Eigen/Dense>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

typedef sensor_msgs::msg::PointCloud2 PointCloud2;
typedef std::shared_ptr<PointCloud2> PointCloud2Ptr;
typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::Matrix3d Matrix3d;

using namespace std;

string left_cloud_topic = "/livox/lidar_3";
string right_cloud_topic = "/livox/lidar_";
string combined_cloud_topic = "/combined/livox/lidar";

double x_distance_to_center = 0.0; // meters
double y_distance_to_center = 0.0; // meters
double angle_c = 20.0; // degrees, clockwise rotation

PointCloud2Ptr left_cloud;
PointCloud2Ptr right_cloud;

MatrixXd left_xyz;
MatrixXd right_xyz;

std::mutex left_cloud_mutex;
std::mutex right_cloud_mutex;

Matrix3d rotationMatrix(double degrees) {
    double radians = degrees * M_PI / 180.0;
    Matrix3d R;
    R << std::cos(radians), -std::sin(radians), 0,
         std::sin(radians), std::cos(radians), 0,
         0, 0, 1;
    return R;
}

MatrixXd rotatePointCloud(const MatrixXd &xyz, double degrees, double x0, double y0, double z0) {
    // Rotate the points
    MatrixXd rotated = xyz * rotationMatrix(degrees);
    // Translate the points 
    return rotated.rowwise() + Eigen::RowVector3d(x0, y0, z0);
}

MatrixXd create_cloud(const PointCloud2Ptr &msg) {
  const size_t number_of_points = msg->height * msg->width;
  MatrixXd xyz(number_of_points, 3);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z)
  {
      xyz(i, 0) = *iter_x;
      xyz(i, 1) = *iter_y;
      xyz(i, 2) = *iter_z;
  }
  return xyz;
}

void update_cloud(const PointCloud2Ptr &msg, const MatrixXd &xyz) {
  const size_t number_of_points = msg->height * msg->width;
  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = xyz(i, 0);
        *iter_y = xyz(i, 1);
        *iter_z = xyz(i, 2);
    }
}

void right_PointCloud2Callback(const PointCloud2Ptr &msg)
{
  MatrixXd xyz = create_cloud(msg);
  MatrixXd rotated_xyz = rotatePointCloud(xyz, angle_c, x_distance_to_center, y_distance_to_center, 0.0);
  update_cloud(msg, rotated_xyz);
  // Create a lock and update the right_cloud
  std::lock_guard<std::mutex> lock(right_cloud_mutex);
  right_cloud = msg;
  right_xyz = rotated_xyz;
  right_cloud_mutex.unlock();
}

void left_PointCloud2Callback(const PointCloud2Ptr &msg)
{
  MatrixXd xyz = create_cloud(msg);
  MatrixXd rotated_xyz = rotatePointCloud(xyz, -angle_c, -x_distance_to_center, y_distance_to_center, 0.0);
  update_cloud(msg, rotated_xyz);
  // Create a lock and update the left_cloud
  std::lock_guard<std::mutex> lock(left_cloud_mutex);
  left_cloud = msg;
  left_xyz = rotated_xyz;
  left_cloud_mutex.unlock();
}

PointCloud2Ptr combine_lidar() {

  //Create locks for the left and right clouds
  std::lock_guard<std::mutex> left_lock(left_cloud_mutex, std::adopt_lock);
  std::lock_guard<std::mutex> right_lock(right_cloud_mutex, std::adopt_lock);

  // Check if the clouds are not empty
  if (left_cloud == nullptr || right_cloud == nullptr) {
    return nullptr;
  }

  // Combine the clouds
  PointCloud2Ptr combined_cloud = left_cloud;
  combined_cloud->header.stamp = rclcpp::Clock().now();
  combined_cloud->width = left_cloud->width + right_cloud->width;
  combined_cloud->height = 1;

  // Combine the xyz matrices
  MatrixXd combined_xyz(left_xyz.rows() + right_xyz.rows(), 3);
  combined_xyz << left_xyz, right_xyz;

  // Update the combined cloud
  update_cloud(combined_cloud, combined_xyz);

  // Return the combined cloud
  return combined_cloud;
}


using namespace std::chrono_literals;

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

// class CombineLivox : public rclcpp::Node
// {
//   public:
//     CombineLivox()
//     : Node("combine_livox")
//     {
//       // Setup the callback groups
//       left_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//       right_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

//       // Setup the subscription options
//       auto leftopts = rclcpp::SubscriptionOptions();
//       leftopts.callback_group = left_group;
//       auto rightopts = rclcpp::SubscriptionOptions();
//       rightopts.callback_group = right_group;

//       // Setup the subscription and publisher
//       left_cloud_sub = this->create_subscription<PointCloud2>(left_cloud_topic, 10, left_PointCloud2Callback, leftopts);
//       right_cloud_sub_ = this->create_subscription<PointCloud2>(right_cloud_topic, 10, right_PointCloud2Callback, rightopts);
//       combined_cloud_pub_ = this->create_publisher<PointCloud2>(combined_cloud_topic, 10);

//       // Setup the timer
//       auto timer_callback = [this]() -> void {
//         PointCloud2Ptr combined_cloud = combine_lidar();
//         if (combined_cloud != nullptr) {
//           this->combined_cloud_pub_->publish(*combined_cloud);
//         }
//       };
//       auto timer = this->create_wall_timer(50ms, timer_callback);
//     }

//   private:
//     rclcpp::CallbackGroup::SharedPtr left_group;
//     rclcpp::CallbackGroup::SharedPtr right_group;
//     rclcpp::Subscription<PointCloud2>::SharedPtr left_cloud_sub;
//     rclcpp::Subscription<PointCloud2>::SharedPtr right_cloud_sub_;
//     rclcpp::Publisher<PointCloud2>::SharedPtr combined_cloud_pub_;
// };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::executors::MultiThreadedExecutor executor;
  // auto node = std::make_shared<CombineLivox>();
  // executor.add_node(node);
  // executor.spin();
  // rclcpp::shutdown();
  return 0;
}