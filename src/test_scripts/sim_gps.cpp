/*
A basic ROS node that publishes GPS data to the /RX/gps topic.
*/

#include <chrono>
#include <csignal>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/lat_lon_head.hpp"

void SigHandle(int sig) 
{
  if (sig == SIGINT) {} // to ignore warning
  RCLCPP_INFO(rclcpp::get_logger("combine_livox"), "Shutting down combine_livox node...");
  rclcpp::shutdown();
}

class SimGPS : public rclcpp::Node
{
    public:
        SimGPS() : Node("sim_gps")
        {
            using namespace std::chrono_literals;
            gps_pub_ = this->create_publisher<interfaces::msg::LatLonHead>("/RX/gps", 10);
            timer_ = this->create_wall_timer(1000ms, std::bind(&SimGPS::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            interfaces::msg::LatLonHead msg;
            msg.latitude = 33.64094057570441;
            msg.longitude = -117.82929527244504;
            msg.heading = 0;
            msg.header.stamp = rclcpp::Clock().now();
            gps_pub_->publish(msg);
        }

        rclcpp::Publisher<interfaces::msg::LatLonHead>::SharedPtr gps_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, SigHandle);
    auto node = std::make_shared<SimGPS>();
    RCLCPP_INFO(node->get_logger(),"Starting the gps_sim node...");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(),"Shutting down the gps_sim node...");
    rclcpp::shutdown();
    return 0;
}