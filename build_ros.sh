echo "Building ROS nodes"
currentDirectory=$(pwd)
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
