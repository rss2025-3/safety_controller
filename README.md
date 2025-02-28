# Safety Controller

## Overview
The **Safety Controller** is a ROS2 package that stops the robot if a collision is about to happen. If you want to see it working, just run the lab2 testcases then launch this node.

## Running the Safety Controller
To launch the safety controller, run in ~/racecar_ws:
```sh
ros2 launch safety_controller safety_controller.launch.xml
```

## Code Modifications
There are **TODO** comments in the code that indicate necessary changes for the controller to work on the actual robot. These modifications are located in:
- `params.yaml`
- `safety_controller.py`

## Lab2 might be broken?
Also your lab2 may not work with the new ROS update, if so I've uploaded my fixed version. 

