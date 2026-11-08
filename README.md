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

The sections below describe the legacy ROS 1 packages in `ros/`.

## Doc

Doc includes tutorials about how to build your own robot arm. Contents contain the whole process starting from
how to do assembly in Solidworks in order to make urdf generation easier to use MoveIt! setup assistant to build
your own robot arm MoveIt! package.

## mechanical

Mechanical includes 3DParts and Assembly two folders. 3DParts folder maintains all the files including Solidworks file,
3Dprinting file and drawing that belong to individual piece. Assembly holds models for the whole arm and each link.

## ros

ros folder keeps all ROS-related packages to the handy arm, which includes description, control, gazebo and MoveIt! package.
There are two versions of Handy, one is the normal model which the other one is mesh-simplified model. Each has the four above packages.

### dependencies

1. If you'd like to use finalarm_gazebo package, you should git clone balabala packages to your workspace since gazebo has dependencies on these packages if you want to load controllers in gazebo.

2. If you just need finalarm_control, finalarm_description and finalarm_moveit_config packages, in order to compile all packages out, you additionally need a package named dynamixel_motor which applies designed controllers for dynamixel motors.

### real world motion planning with Handy

Ensure that the current user is part of the `dialout` group.
If you get permission issues connecting to `/dev/ttyUSB0`, run the following and logout+login.

```bash
sudo usermod -aG dialout $USER
```

Launch handy with the following commands:

```bash
# start a node responsible for managing loaded controllers
roslaunch finalarm_control controller_manager.launch

# load configuration for individual controller
roslaunch finalarm_control start_controller.launch

# publish integrated information of controllers and transformations between links
roslaunch finalarm_description robot_state_pub.launch

# node for motion planning, collision checking, etc. jobs
roslaunch finalarm_moveit_config move_group.launch

# rviz for visualization
roslaunch finalarm_moveit_config moveit_rviz.launch

# launch the file you write for Handy to do something like picking object
roslaunch handy_experiment pick.launch
```

You can run the above using `docker-compose` instead:

```bash
docker-compose up

# node for motion planning, collision checking, etc. jobs
roslaunch finalarm_moveit_config move_group.launch

# rviz for visualization
roslaunch finalarm_moveit_config moveit_rviz.launch
```

Read [docs/docker.md](docs/docker.md) for more details.

## launchfiles

Here's a list of launch files that can be tested without access to the physical robot.

```bash
# check for urdf loading
roslaunch finalarm_description display.launch
# check robot position topics
roslaunch finalarm_description gazebo.launch
roslaunch finalarm_gazebo robot_world.launch
roslaunch finalarm_gazebo robot_world.launch control_mode:=position
# check operation of robot
roslaunch finalarm_moveit_config demo.launch
# check gazebo and moveit operate together
roslaunch finalarm_gazebo robot_world_moveit.launch

roslaunch finalarm_simplified_description display.launch
roslaunch finalarm_simplified_description gazebo.launch
roslaunch finalarm_simplified_gazebo robot_world.launch
roslaunch finalarm_simplified_gazebo robot_world_urdf.launch
roslaunch finalarm_simplified_gazebo robot_world_xacro.launch
roslaunch finalarm_simplified_moveit_config demo.launch
roslaunch finalarm_simplified_gazebo robot_world_moveit.launch
```

These are useful for manual testing, at least until some form of automated testing is implemented.
