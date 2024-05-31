echo "Building ROS nodes"
currentDirectory=$(pwd)
export ROS_PACKAGE_PATH=/home/li/catkin_ws/src/V_SLAM_LZQ/Examples/ROS/ORB_SLAM3:$ROS_PACKAGE_PATH
cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j5
cd ${currentDirectory}
echo "------------->>>>>>>>>>>>."${currentDirectory}
source build.sh
cd ${currentDirectory}
source lib_copy.sh
