# Scalar_Punk
Robot eyes emotions that can control by ros2 service


-Clone in your work space 
```
git clone https://github.com/gyiptgyipt/ScalarPunk.git
```

-And build
```
colcon build
```

```
source install/setup.bash
```

and run the node 

```
ros2 run scalarpunk scalarpunk
```


Command for servicec call for test
<!-- ```
ros2 topic pub -r 1 /robot_emotion std_msgs/msg/String "data: 'angry'" --once
``` -->

eg. 
```
ros2 service call /robot_emotion_command scalarpunk_interfaces/srv/EmotionCommand "{emotion: 'happy'}"
```

Emotion Parameter list that can use: 
```
happy
angry
look_left
look_right
charging
smile
cry
wakeup
```



![Emotions_Example](/images/emotions.png)


TODO: Clean Code