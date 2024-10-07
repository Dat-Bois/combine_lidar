#pragma once

#include <cmath>
#include <mutex>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <occupancy_grid/grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "interfaces/msg/lat_lon_head.hpp"


/*
This is a C++ implementation of the occupancy grid I wrote in python.

Taken from the Python notes:

A class which builds an occupancy grid as the robot moves through the environment.
It relies on an origin lat/lon which is centered at (0,0).

To update the grid, it takes in the current lat/lon, current heading, and the lidar data.

The lidar data is reduced to only points where z < 0, z > -1.5, x < 30, and y < 30, y > -30.
This makes it essentially a 30m x 30m x 1.5m box positioned directly in front of the robot (since the lidars are mounted on the front of the robot).
The 1.5m height is to remove points on the ground and above the robot. The 30m x 30m box is to remove points that are too far away from the robot.
The 1.5m is then also condensed into one plane by overlaying the points on the x-y plane.

The grid is a dictionary where the key is the x,y coordinate and the value is the occupancy value.
As the robot moves, the grid can expand in any direction.

The grid is initialized with -1 in each cell to represent unknown occupancy.
The higher the value of a cell (max 100), the more certain we are that the cell is occupied.
When the robot moves, the local grid is overlayed onto the global grid at the robot's current position and heading.

*/

typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::Matrix2d Matrix2d;
typedef Eigen::Vector2d Vector2d;

using namespace std;

class OccupancyGrid
{
    public:
        OccupancyGrid(double cell_size);
        OccupancyGrid(vector<double> origin, double cell_size);

        void update_gps(const interfaces::msg::LatLonHead::SharedPtr msg);
        void update_grid(sensor_msgs::msg::PointCloud2::SharedPtr msg);

        Grid get_grid();

    private:

        Grid grid;

        vector<double> origin = {0, 0};
        vector<double> position = {0, 0, 0};

        double cell_size;
        double lidar_offset = 0.5; // meters

        double sin_deg(double deg);
        double cos_deg(double deg);
        const double PI = 3.14159265358979323846;

        std::mutex gps_mutex;
        void set_origin(vector<double> origin);

        vector<int> global_to_local(double lat, double lon);
        vector<double> local_to_global(int x, int y);

        MatrixXd clean_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        Grid process_local_grid(const MatrixXd &xyz);
        Grid orient_local_grid(Grid &local_grid, double heading, vector<int> local_origin);

        std::mutex grid_mutex;
        void update_grid(double lat, double lon, double heading, sensor_msgs::msg::PointCloud2::SharedPtr msg);

};