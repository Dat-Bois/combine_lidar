#pragma once

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>

typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::Matrix3d Matrix3d;

Matrix3d rotationMatrix(double degrees);
MatrixXd rotatePointCloud(const MatrixXd &xyz, double degrees, double x0, double y0, double z0);
MatrixXd create_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
void update_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const MatrixXd &xyz);

#endif // MATH_UTILS_HPP
