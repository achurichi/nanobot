# nanobot

Learning ROS2 on a Jetson Nano Bot

![Nanobot](./3d_parts/nanobot.png)

## Scripts

The `scripts` folder contains useful scripts to manage the project:

- **`build.sh`**: Builds everything needed to run the real robot or the simulation.
- **`start.sh`**: Starts the robot or the simulation (if run with the `sim` parameter).
- **`clean.sh`**: Removes the files generated during the build process.

## Controlling the Robot

To move the robot using keyboard teleoperation, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_controller/cmd_vel_unstamped
```
