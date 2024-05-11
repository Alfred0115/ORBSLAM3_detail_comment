mkdir log
cat ll.log |grep "LoopClosing">log/loopclosing.log
cat ll.log |grep "LocalMapping-->>">log/localmap.log
cat ll.log |grep "Tracking-->>">log/trace.log
cat ll.log |grep "Optimizer-->>">log/optimizer.log
echo "get log"
