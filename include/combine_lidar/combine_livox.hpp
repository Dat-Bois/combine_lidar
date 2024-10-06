#pragma once

#ifndef COMBINE_LIVOX_HPP
#define COMBINE_LIVOX_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <mutex>

typedef sensor_msgs::msg::PointCloud2 PointCloud2;
typedef std::shared_ptr<PointCloud2> PointCloud2Ptr;

class CombineLivox : public rclcpp::Node
{
public:
    CombineLivox();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr left_group;
    rclcpp::CallbackGroup::SharedPtr right_group;
    rclcpp::Subscription<PointCloud2>::SharedPtr left_cloud_sub_;
    rclcpp::Subscription<PointCloud2>::SharedPtr right_cloud_sub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr combined_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr comb_lidar_status_pub_;
};

void SigHandle(int sig);
PointCloud2Ptr combine_lidar();
void right_PointCloud2Callback(const PointCloud2Ptr msg);
void left_PointCloud2Callback(const PointCloud2Ptr msg);

#endif // COMBINE_LIVOX_HPP
