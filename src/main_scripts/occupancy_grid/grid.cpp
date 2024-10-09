#include "occupancy_grid/grid.hpp"

using namespace std;


Grid::Grid(uint8_t default_value) : default_value(default_value) {}

void Grid::adjust_ranges(int x, int y) {
    x_range[0] = min(x_range[0], x);
    x_range[1] = max(x_range[1], x);
    y_range[0] = min(y_range[0], y);
    y_range[1] = max(y_range[1], y);
}

uint8_t Grid::get_value(int x, int y) const {
    auto it = grid.find({x, y});
    if (it == grid.end()) {
        return default_value;
    }
    return it->second;
}

void Grid::set_value(int x, int y, uint8_t value) {
    adjust_ranges(x, y);
    value = min(max_value, max(min_value, value));
    grid[{x, y}] = value;
}

void Grid::increment_value(int x, int y, int value) {
    adjust_ranges(x, y);
    int new_value = get_value(x, y) + value;
    set_value(x, y, new_value);
}

void Grid::add_grid(Grid grid) {
    for (const auto &e : grid) {
        pair<int, int> xy = e.first;
        uint8_t value = e.second;
        increment_value(xy.first, xy.second, value);
    }
}

interfaces::msg::Grid Grid::get_msg() {
    interfaces::msg::Grid msg;
    msg.x_min = x_range[0];
    msg.x_max = x_range[1];
    msg.y_min = y_range[0];
    msg.y_max = y_range[1];
    for (const auto &e : grid) {
        interfaces::msg::Cell cell;
        cell.x_coord = e.first.first;
        cell.y_coord = e.first.second;
        cell.value = e.second;
    }
    return msg;
}