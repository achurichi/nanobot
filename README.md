# Nanobot

## About This Project

This project is designed for learning ROS 2 on a Jetson Nano-powered robot. It includes various features for perception, control, and simulation, such as:

- **LiDAR** for environment mapping and obstacle detection.
- **Depth Camera** for 3D vision and object recognition.
- **Motor Control with ROS 2 Control** for precise movement and actuation.
- **Navigation** with path planning and obstacle avoidance.
- **Gazebo Simulation** for testing and development in a virtual environment.

![Nanobot](./3d_parts/nanobot.png)

## Scripts

The `scripts` folder contains useful scripts to manage the project:

- **`build.sh`**: Builds everything needed to run the real robot or the simulation (if run with the `sim` parameter).
- **`clean.sh`**: Removes the files generated during the build process.
- **`start.sh`**: Starts the robot or the simulation (if run with the `sim` parameter).
- **`monitor.sh`**: Runs rviz and rqt_image_view.
- **`magnetometer_calibration.py`**: Run it to get the calibration values for the magnetometer. Then update the values in `/ros_ws/src/nanobot_imu/config/imu.yaml`.

## Controlling the Robot

You can control the robot in three different ways:

- **Using a Joystick**  
   If you have a joystick connected via USB, the robot can be controlled directly through it.

- **Using the Web App**
  You can control the robot through a web interface. Open a browser and navigate to:  
   `http://<jetson-nano-ip>:5173`

- **Using the Keyboard**  
   To control the robot using the keyboard, run the following command:
  ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
  ```

## Configure the Power Button

Ensure the power button is connected correctly to the Jetson Nano:

- Connect the power button to **Pin 1** and **Pin 2** of the header pins.
- Jumper **Pin 7** and **Pin 8** of the button header pins to disable the auto power on.

Open the `logind.conf` file to adjust the power button settings:

```bash
sudo nano /etc/systemd/logind.conf
```

Add the following lines to the file:

```bash
NAutoVTs=1
ReserveVT=1
HandlePowerKey=poweroff
```

The most reliable way to configure the power button to turn on/off the Jetson Nano is by disabling the graphical user interface (GUI), as GNOME interferes with the button behavior defined in `logind.conf`.

To boot with no GUI:

```bash
sudo systemctl set-default multi-user.target
```

Auto-login is required to allow SSH connections after the system boots up. To enable it, create a directory for custom configurations:

```bash
sudo mkdir /etc/systemd/system/getty@tty1.service.d/
```

Next, open the override configuration file:

```bash
sudo nano /etc/systemd/system/getty@tty1.service.d/override.conf
```

Add the following content to automatically log in your user:

```bash
[Service]
ExecStart=
ExecStart=-/sbin/agetty --noissue --autologin <user-name> %I $TERM
Type=idle
```

Replace `<user-name>` with your actual username.

Finally, reboot the system to apply the changes.
