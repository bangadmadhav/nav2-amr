# Nav2 Autonomous Mobile Robot (ROS2)

A ROS2-based autonomous mobile robot using the Nav2 stack for mapping, localization, planning, and control.

## What it does

- Builds a map using SLAM
- Localizes using AMCL
- Plans paths using multiple planners (NavFn, Theta*, Smac)
- Executes motion using different controllers (DWB, RPP, MPPI, Graceful)
- Navigates to goal positions in a structured environment

## System Overview

The system follows a standard navigation pipeline:
<br /><br />
(for Mapping)<br />
Load world → run SLAM → manually drive robot → generate and save map
<br /><br />
(for Navigation)<br />
Load world + saved map → localization → global planning → planners & controllers → execution
<br /><br />
All components are configured and integrated within the Nav2 framework.

---

## Run

From the `ros2_ws` directory:

```bash
# Terminal 1 - Launch Gazebo, Rviz, spawn Robot & intialize controllers
source install/setup.bash
ros2 launch robot_bringup display.launch.py world_name:=world1 
```

### For Mapping
```bash
# Terminal 2 - Run SLAM launch file
source install/setup.bash
ros2 launch robot_bringup slam.launch.py

#Terminal 3 - Activate slam_toolbox Lifecycle node and run teleop_twist_keyboard
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4 - To save the map
ros2 run nav2_map_server map_saver_cli -f map_iter
```
### For Navigation
```bash
# Terminal 2 -  Run nav2 Launch file
source install/setup.bash
ros2 launch robot_bringup nav2.launch.py map_name:=world1 planner:=navfn controller:=rpp
```

---

### Notes
* Multiple planners and controllers are configured for experimentation
* Custom nodes handle message/frame compatibility within the pipeline
* Maps and worlds are included for testing different scenarios

### Demo
- AMR Navigation demo:<br />
  Video: https://youtu.be/gft94JqwDNI <br />
  Image: ![AMR Navigation in Rviz](/assets/navigation_rviz.png)
  
