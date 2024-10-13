#pragma once
#include <vector>
#include "interfaces/msg/cell.hpp"
#include "interfaces/msg/grid.hpp"
#include <parallel_hashmap/phmap.h>


using namespace std;

class Grid
{
    /*
    The issue with using an array as a grid is that even if the grid is sparse, it still takes up a lot of memory. 
    (Ex. if we wanted just an L shape, it would still be a full square).
    To solve this, we can use a hashtable where the key is the x,y coordinate and the value is the occupancy value.
    To avoid using large ints, we will store just uint8_t values, where 20 and below is treated as negative.
    */

    public:
        Grid(uint8_t default_value = 20);

        // Copy constructor
        Grid(const Grid &other_grid) :  max_value(other_grid.max_value), zero_point(other_grid.zero_point), 
                                        min_value(other_grid.min_value), default_value(other_grid.default_value),
                                        x_range(other_grid.x_range), y_range(other_grid.y_range)
        {
            for (const auto &e : other_grid.grid) {
                pair<int, int> xy = e.first;
                uint8_t value = e.second;
                grid[{xy.first, xy.second}] = value;
            }

        }

        // Delete grid
        // ~Grid() {grid.clear();}

        uint8_t operator()(int x, int y) const {return get_value(x, y);}
        void operator()(int x, int y, uint8_t value) {set_value(x, y, value);}

        void increment_value(int x, int y, int value);

        void add_grid(const Grid &other_grid);

        vector<int> get_x_range() {return x_range;}
        vector<int> get_y_range() {return y_range;}
        bool is_zero(int x, int y) {return get_value(x, y) == zero_point;}
        int get_size() {return grid.size();}

        interfaces::msg::Grid get_msg();
        
        class Iterator {
            public:
                using map_iterator = phmap::flat_hash_map<std::pair<int, int>, uint8_t>::iterator;
                Iterator(map_iterator iter) : map_iter(iter) {}
                bool operator==(const Iterator &other) const {
                    return map_iter == other.map_iter;
                }
                bool operator!=(const Iterator &other) const {
                    return map_iter != other.map_iter;
                }
                Iterator &operator++() {
                    ++map_iter;
                    return *this;
                }
                const pair<pair<int, int>, uint8_t> operator*() const {
                    return *map_iter;
                }
                std::pair<const std::pair<int, int>, uint8_t>* operator->() const {
                    return &(*map_iter);
                }
            private:
                map_iterator map_iter;
        };
        Iterator begin() {return Iterator(grid.begin());}
        Iterator end() {return Iterator(grid.end());}

    private:
        phmap::flat_hash_map<pair<int, int>, uint8_t> grid;
        uint8_t max_value = 255;
        uint8_t zero_point = 20;
        uint8_t min_value = 0;
        uint8_t default_value;

        vector<int> x_range = {0, 0}; // [min, max]
        vector<int> y_range = {0, 0}; // [min, max]

        void adjust_ranges(int x, int y);
        void set_value(int x, int y, int value);
        uint8_t get_value(int x, int y) const;

};