mkdir z_log
cat ll.log |grep "LoopClosing">z_log/loopclosing.log
cat ll.log |grep "LocalMapping-->>">z_log/localmap.log
cat ll.log |grep "Tracking-->>">z_log/trace.log
cat ll.log |grep "Optimizer-->>">z_log/optimizer.log
echo "get log"
