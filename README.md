
Make sure to move the temp/interfaces package out of this package and into your workspace level src.

Some commands:

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

ros2 run --prefix 'gdb -ex run --args' combine_lidar combine_lidar_node

(gdb) thread apply all bt

