rosbag play ../MH_01_easy.bag
# mkdir z_log
cat z_log/ll.log |grep "LoopClosing">z_log/loopclosing.log
cat z_log/ll.log |grep "LocalMapping-->>">z_log/localmap.log
cat z_log/ll.log |grep "Tracking-->>">z_log/trace.log
cat z_log/ll.log |grep "Optimizer-->>">z_log/optimizer.log
cat z_log/ll.log |grep "robot_pose">z_log/pose.log

echo "get log"
