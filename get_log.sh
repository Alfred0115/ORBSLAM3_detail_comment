# rosbag play ../MH_01_easy.bag
# mkdir z_log
filename=$1
cat z_log/${filename}.log |grep "LoopClosing">z_log/${filename}_loopclosing.log
cat z_log/${filename}.log |grep "LocalMapping-->>">z_log/${filename}_localmap.log
cat z_log/${filename}.log |grep "Tracking-->>">z_log/${filename}_trace.log
cat z_log/${filename}.log |grep "Optimizer-->>">z_log/${filename}_optimizer.log
cat z_log/${filename}.log |grep "robot_pose">z_log/${filename}_pose.log
cat z_log/${filename}.log |grep "System-->>">z_log/${filename}_system.log

echo "get log"
