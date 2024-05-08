echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM3
mkdir build_ros
cd build_ros
cmake .. -DROS_BUILD_TYPE=Release
make -j
