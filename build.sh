echo "Configuring and building Thirdparty/DBoW2 ..."
#https://blog.csdn.net/u014374826/article/details/132013820
# cd Thirdparty/DBoW2
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ../../g2o

# echo "Configuring and building Thirdparty/g2o ..."

# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ../../Sophus

# echo "Configuring and building Thirdparty/Sophus ..."

# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ../../../

# echo "Uncompress vocabulary ..."

# cd Vocabulary
# tar -xf ORBvoc.txt.tar.gz
# cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j6
cp lib/libORB_SLAM3.so                  ~/run_lib
