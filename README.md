# turtlebot_launcher
ROS2 service to launch ping pong ball using flywheel-based launcher. Meant for Turtlebot3 platform.

Written for NUS CDE2310 AY24/25. 

# Dependencies: 
* pigpio
* rclpy
* std_srvs

# Installation:
* Create target folder within your workspace. (In this case, ros2_ws.)

```
cd ~/ros2_ws
mkdir -p src/turtlebot_launcher
cd src/turtlebot_launcher
```

* Git clone repository into folder

```
git clone https://github.com/cde2310grp6/ball_launcher.git
```

* Navigate back to workspace and build using colcon

```
cd ~/ros2_ws
colcon build --packages-select turtlebot_launcher
```

# Running the service:

```
ros2 run turtlebot_launcher launcher_service 
```

# Acknowledgements:
* Instructors of CDE2310:
- Mr Chew Wanlong, Nicholas
- Mr Ee Wei Han, Eugene
- Mr Royston Shieh Teng Wei
- Mr Soh Eng Keng

* Teaching Assistants of CDE2310
