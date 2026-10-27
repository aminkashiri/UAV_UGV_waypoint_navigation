# UAV-UGV Waypoint Navigation

ROS 2 implementation of the hardware experimental framework for the paper:

> C. I. Er, A. Kashiri, and Y. Yazıcıoğlu, "RSPECT: Robust and Scalable Planner for Energy-Aware Coordination of UAV-UGV Teams in Aerial Monitoring," *Journal of Intelligent & Robotic Systems*, 2026.

The framework executes offline UAV-UGV plans on physical robots. Each team consists of a Crazyflie 2.1 UAV and a TurtleBot 4 UGV that serves as a mobile charging station. The UGV transports the UAV to a release point, the UAV takes off and visits its assigned monitoring waypoints, and it then lands on the UGV at a collect point before the next tour begins. Both vehicles are localized by an OptiTrack motion capture system.

<p align="center">
  <a href="https://youtu.be/jQkGfwVTdBc">
    <img src="https://img.youtube.com/vi/jQkGfwVTdBc/maxresdefault.jpg" alt="Video of the hardware experiments" width="640">
  </a>
  <br>
  <em>Multi-team hardware experiments reported in the paper (click to watch).</em>
</p>

This repository contains the initial single-team version of the framework. The multi-team experiments reported in the paper were conducted with an extended version built on top of it, which is not included here. The RSPECT planner is also outside the scope of this repository; plan waypoints are specified directly in the nodes.

## Components

- `uav/optitrack_feed_node.py`: Streams motion capture poses to the Crazyflie over the Crazyradio as external pose measurements for the onboard extended Kalman filter.
- `uav/uav_node.py`: Executes UAV tours with the cflib high-level commander (takeoff, waypoint navigation, landing). The landing target is taken from the UGV's live motion capture pose.
- `uav/cf_driver.py`: Manages the Crazyflie connection and state estimator configuration.
- `ugv/ugv_node.py`: Controls the TurtleBot 4 through a finite state machine (move to release, trigger takeoff, move to collect, wait for landing) built on a proportional go-to-goal controller with heading alignment.

UAV-UGV coordination is handled through two ROS 2 services: the UGV calls `drone_takeoff` upon reaching a release point, and the UAV calls `drone_landed` after landing on the UGV.

## Usage

Requirements: ROS 2, [cflib](https://github.com/bitcraze/crazyflie-lib-python), a Crazyradio, and a motion capture bridge publishing `geometry_msgs/PoseStamped` on `/Robot_1/pose` (UAV) and `/Robot_2/pose` (UGV).

```bash
colcon build --packages-select uav_ugv_nav
source install/setup.bash

ros2 run uav_ugv_nav ugv_node   # UGV control
ros2 run uav_ugv_nav uav_node   # motion capture feed and UAV control
```

## Citation

If you use this work, please cite:

```bibtex
@article{er2026rspect,
  title={Rspect: Robust and scalable planner for energy-aware coordination of uav-ugv teams in aerial monitoring},
  author={Er, Cahit Ikbal and Kashiri, Amin and Yaz{\i}c{\i}o{\u{g}}lu, Yasin},
  journal={Journal of Intelligent \& Robotic Systems},
  year={2026},
  publisher={Springer}
}
```
