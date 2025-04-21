# turtlebot_launcher
ROS2 service to launch ping pong ball using flywheel-based launcher. Meant for Turtlebot3 platform.

Written for NUS CDE2310 AY24/25. 

# Dependencies: 
* RPi.GPIO
* rclpy
* std_srvs

# Installation:
1. Navigate to source folder within your workspace. (In this case, 2310_workspace.)

```
cd ~/2310_workspace/src
```

2. Git clone repository into folder

```
git clone https://github.com/cde2310grp6/turtlebot_launcher.git
```

3. Navigate back to workspace and build using colcon

```
cd ~/2310_workspace
colcon build --packages-select turtlebot_launcher
```

# Running the service:

```
ros2 run turtlebot_launcher launcher_service 
```

# Calling the service through cmd line
For debugging purposes, it may be useful to be able to call the service via command line  
Ensure the above service is running, afterwards in another terminal: 
```
ros2 service call /launch_ball std_srvs/srv/Trigger "{}"
```

# Calling the service through another node
Include this in the init of the node:
```
self.launcher_service = self.create_client(Trigger, 'launch_ball')
```
Sample service call and callback functions within node:
```
 def launch_now(self):
     self.get_logger().info("calling launcher")
     self.req = Trigger.Request()
     # Ensure service is available
     if not self.launcher_service.wait_for_service(timeout_sec=5.0):
         self.get_logger().warning('launcher service not available')
         return
     # Call the service asynchronously and handle the response when ready
     future = self.launcher_service.call_async(self.req)
     future.add_done_callback(self.launch_callback)

 def launch_callback(self, future):
     try:
         response = future.result()  # Get the response from the service call
         # Update saving_in_progress to continue saving the next target
         self.saving_in_progress = False
     except Exception as e:
         self.get_logger().error(f"launch service call failed: {e}")
```

# Updating:

Enter the turtlebot_launcher folder and git pull, then build with colcon

```
cd ~/2310_workspace/src/turtlebot_launcher && git pull
cd ~/2310_workspace && colcon build --packages-select turtlebot_launcher
```

# Acknowledgements:
* Instructors of CDE2310:
    * Mr Chew Wanlong, Nicholas
    * Mr Ee Wei Han, Eugene
    * Mr Royston Shieh Teng Wei
    * Mr Soh Eng Keng

* Teaching Assistants of CDE2310
