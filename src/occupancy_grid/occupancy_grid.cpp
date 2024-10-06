#include "occupancy_grid/occupancy_grid.hpp"


using namespace std;

OccupancyGrid::OccupancyGrid(vector<double> origin, double cell_size) : origin(origin), cell_size(cell_size) {}

double OccupancyGrid::sin_deg(double d) {
	if (fmod(d, 180) == 0) { return 0; }
	return sin((d * PI) / 180);
}

double OccupancyGrid::cos_deg(double d) {
	if (fmod(d, 90) == 0 && ((int)d/90)%2==1) { return 0; }
	return cos((d * PI) / 180);
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

MatrixXd OccupancyGrid::clean_lidar(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const size_t number_of_points = msg->height * msg->width;
    MatrixXd xyz(number_of_points, 3);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        if (    iter_z[i] > 0 || iter_z[i] < -1.5 || 
                iter_y[i] > 15 || iter_y[i] < -15 || 
                iter_x[i] > 30) 
        {   
            xyz(i, 0) = *iter_x;
            xyz(i, 1) = *iter_y;
            xyz(i, 2) = *iter_z;
        }
    }
    return xyz;
}

Grid OccupancyGrid::process_local_grid(MatrixXd xyz) {
    Grid local_grid;
    
    // Increment the value of the cell by 5 for each point
    for (int i = 0; i < xyz.rows(); i++) {
        int x = floor((-1 * xyz(i,1)) / cell_size);
        int y = floor((xyz(i,0)+lidar_offset) / cell_size);
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