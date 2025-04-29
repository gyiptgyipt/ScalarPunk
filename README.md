# Scalar_Punk
Robot eyes emotions that can control by ros2 topic

-build in your work space
and run the node 

```
ros2 run scalarpunk scalarpunk
```


Command for servicec call for test
<!-- ```
ros2 topic pub -r 1 /robot_emotion std_msgs/msg/String "data: 'angry'" --once
``` -->

ros2 service call /robot_emotion_command scalarpunk_interfaces/srv/EmotionCommand "{emotion: 'happy'}"