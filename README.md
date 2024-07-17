Some commands:


colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

ros2 run --prefix 'gdb -ex run --args' combine_lidar combine_lidar_node

(gdb) thread apply all bt
