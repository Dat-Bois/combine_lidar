#include "occupancy_grid/occupancy_grid.hpp"

using namespace std;

OccupancyGrid::OccupancyGrid(double cell_size) : cell_size(cell_size) {}
OccupancyGrid::OccupancyGrid(vector<double> origin, double cell_size) : origin(origin), cell_size(cell_size) {}

double OccupancyGrid::sin_deg(double d) {
	if (fmod(d, 180) == 0) { return 0; }
	return sin((d * PI) / 180);
}

double OccupancyGrid::cos_deg(double d) {
	if (fmod(d, 90) == 0 && ((int)d/90)%2==1) { return 0; }
	return cos((d * PI) / 180);
}

void OccupancyGrid::set_origin(vector<double> origin) {
    this->origin = origin;
}

vector<int> OccupancyGrid::global_to_local(double lat, double lon) {
    double x = (lat-origin[0]) * 111139;
    double y = (lon-origin[1]) * 111139 * cos_deg(lat);
    return {(int)(x/cell_size), (int)(y/cell_size)};
}

vector<double> OccupancyGrid::local_to_global(int x, int y) {
    double lat = x * cell_size / 111139 + origin[0];
    double lon = y * cell_size / (111139 * cos_deg(lat)) + origin[1];
    return {lat, lon};
}

MatrixXd OccupancyGrid::clean_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const size_t number_of_points = msg->height * msg->width;
    MatrixXd xy(number_of_points, 2);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        if (    iter_z[i] < 0 && iter_z[i] > -1.5 && 
                iter_y[i] < 15 && iter_y[i] > -15 && 
                iter_x[i] < 30) 
        {   
            xy(i, 0) = *iter_x;
            xy(i, 1) = *iter_y;
        }
    }
    return xy;
}

Grid OccupancyGrid::process_local_grid(const MatrixXd &xy) {
    Grid local_grid;
    
    // Increment the value of the cell by 5 for each point
    for (int i = 0; i < xy.rows(); i++) {
        int x = floor((-1 * xy(i,1)) / cell_size);
        int y = floor((xy(i,0)+lidar_offset) / cell_size);
        local_grid.increment_value(x, y, 5);
    }

    // For points within the range, but no points, decrement the value by 10
    vector<int> x_range = local_grid.get_x_range();
    vector<int> y_range = local_grid.get_y_range();
    for (int x = x_range[0]; x <= x_range[1]; x++) {
        for (int y = y_range[0]; y <= y_range[1]; y++) {
            if (local_grid(x, y) == 20) {
                local_grid.increment_value(x, y, -10);
            }
        }
    }
    return local_grid;
}

Grid OccupancyGrid::orient_local_grid(Grid &local_grid, double heading, vector<int> local_origin) {
    /*
    Orients the local grid to match the robot's heading and position.
    The robot's heading is 0 when it is facing North.
    The local grid is oriented with x as right and y as forward.
    */
   double theta = heading * (PI / 180);
   // Define the rotation matrix (2D)
    Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta), cos(theta);

    // Create a new grid to store the rotated values
    Grid oriented_grid;
    for (const auto &e : local_grid) {
        pair<int, int> xy = e.first;
        uint8_t value = e.second;
        // Rotate the x,y coordinates
        Vector2d rotated_xy = R * Vector2d(xy.first, xy.second);
        int x = rotated_xy(0) + local_origin[0];
        int y = rotated_xy(1) + local_origin[1];
        oriented_grid(x, y, value);
    }
    return oriented_grid;
}

void OccupancyGrid::update_gps(const interfaces::msg::LatLonHead::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(gps_mutex);
    if(origin[0] == 0 && origin[1] == 0) {
        set_origin({msg->latitude, msg->longitude});
    }
    position = {msg->latitude, msg->longitude, msg->heading};
}

void OccupancyGrid::update_grid(double lat, double lon, double heading, sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    vector<int> local_origin = global_to_local(lat, lon);
    MatrixXd xy = clean_lidar(msg);
    Grid local_grid = process_local_grid(xy);
    Grid oriented_grid = orient_local_grid(local_grid, heading, local_origin);
    std::lock_guard<std::mutex> lock(grid_mutex);
    grid.add_grid(oriented_grid);
}

void OccupancyGrid::update_grid(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    vector<double> pos;
    {
        std::lock_guard<std::mutex> lock(gps_mutex);
        pos = position;
    }
    if(pos[0]!=0 && pos[1]!=0 && pos[2]!=0) {
        update_grid(pos[0], pos[1], pos[2], msg);
    }
}

Grid OccupancyGrid::get_grid() {
    std::lock_guard<std::mutex> lock(grid_mutex);
    return Grid(grid);
}