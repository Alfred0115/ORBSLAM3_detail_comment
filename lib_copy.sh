currentDirectory=$(pwd)
echo ${currentDirectory}
cd ~
mkdir run_lib
echo "move projext lib to runlib------------------>>>>>>>>>>>>> "
cd ${currentDirectory}
cp Thirdparty/g2o/lib/libg2o.so         ~/run_lib
cp Thirdparty/DBoW2/lib/libDBoW2.so     ~/run_lib
cp lib/libORB_SLAM3.so                  ~/run_lib
# cp Thirdparty/Pangolin/build/*.so     ~/run_lib