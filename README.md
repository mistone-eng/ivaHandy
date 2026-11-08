# ivaHandy

document and code for Handy arm include tutorials, mechanical files and ROS-related packages.

## ROS 2 (Humble) — current

```
third_party/ivaDynamixel   ivaDynamixel for ROS 2 (git submodule, ros2 branch): Dynamixel driver,
                           ros2_control plugin iva_dynamixel_hardware, motor tools and diagnostics
ros2/handy_description     URDF, meshes, ros2_control xacro, servo table (config/hardware.yaml),
                           calibration (config/calibration.yaml: ivaDynamixel init/min/max)
ros2/handy_control         controllers + control.launch.py (hardware layer only)
ros2/handy_moveit_config   MoveIt 2 + MoveIt Servo configuration and launch files
ros2/handy_bringup         demo.launch.py: full stack (control + MoveIt + RViz + Servo)
testing/handy_examples     Servo demo / jog scripts
testing/handy_calibrate    joint zero-offset and range calibration (no ROS needed)
scripts/handy.sh           `handy` shell helper
```

Quick start:

```bash
mkdir -p ~/handy_ws/src && cd ~/handy_ws/src
git clone --recurse-submodules https://github.com/mistone-eng/ivaHandy.git
echo 'source ~/handy_ws/src/ivaHandy/scripts/handy.sh' >> ~/.bashrc && source ~/.bashrc
cd ~/handy_ws && rosdep install --from-paths src --ignore-src -r -y && handy build

handy sim            # full stack on mock hardware
handy demo           # MoveIt Servo demo (second terminal)
handy motors scan    # real arm: info_dump of all 9 servos (no ROS running)
handy calibrate zero # real arm: measure joint zero offsets
handy real test      # real arm, small +/-0.3 rad joint window
```

Already cloned without submodules? `git submodule update --init`.
Real hardware: user in the `dialout` group, and set the U2D2 latency timer to 1 ms
(`echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`).


## Doc

Doc includes tutorials about how to build your own robot arm. Contents contain the whole process starting from
how to do assembly in Solidworks in order to make urdf generation easier to use MoveIt! setup assistant to build
your own robot arm MoveIt! package.

## mechanical

Mechanical includes 3DParts and Assembly two folders. 3DParts folder maintains all the files including Solidworks file,
3Dprinting file and drawing that belong to individual piece. Assembly holds models for the whole arm and each link.

## ROS 1 (legacy)

The original ROS 1 packages, their docker setup and instructions are in
[legacy/ros1/](legacy/ros1/README.md) (not built, kept for reference).
