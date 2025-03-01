# nanobot

Learning ROS2 on a Jetson Nano Bot

![Nanobot](./3d_parts/nanobot.png)

## Scripts

The `scripts` folder contains useful scripts to manage the project:

- **`build.sh`**: Builds everything needed to run the real robot or the simulation.
- **`start.sh`**: Starts the robot or the simulation (if run with the `sim` parameter).
- **`clean.sh`**: Removes the files generated during the build process.

## Controlling the Robot

If you want to control the real robot (not in simulation) from an external machine connected to the same network, remember to set the ROS discovery server by running:

```bash
export ROS_DISCOVERY_SERVER=<jetson-nano-ip>:11811
```

To move the robot using keyboard, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_controller/cmd_vel_unstamped
```
