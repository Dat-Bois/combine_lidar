#include <cmath>
#include "combine_lidar/math_utils.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

Matrix3d rotationMatrix(double degrees) {
    double radians = degrees * (M_PI / 180.0);
    Matrix3d R;
    R << std::cos(radians), -std::sin(radians), 0,
         std::sin(radians), std::cos(radians), 0,
         0, 0, 1;
    return R;
}

MatrixXd rotatePointCloud(const MatrixXd &xyz, double degrees, double x0, double y0, double z0) {
    MatrixXd translated = xyz.rowwise() - Eigen::RowVector3d(y0, x0, z0);
    MatrixXd rotated = translated * rotationMatrix(degrees);
    return rotated;
}

MatrixXd create_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const size_t number_of_points = msg->height * msg->width;
  MatrixXd xyz(number_of_points, 3);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
      xyz(i, 0) = *iter_x;
      xyz(i, 1) = *iter_y;
      xyz(i, 2) = *iter_z;
  }
  return xyz;
}

void update_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const MatrixXd &xyz) {
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
