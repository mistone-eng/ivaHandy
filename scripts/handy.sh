# Handy arm helper -- source this from ~/.bashrc:
#   echo 'source ~/handy_ws/src/ivaHandy/scripts/handy.sh' >> ~/.bashrc && source ~/.bashrc
#
# Usage:
#   handy env                 source ROS + workspace in this shell
#   handy build [args]        build the Handy packages (from the workspace root)
#   handy control [mock|real] hardware layer only: ros2_control + controllers (default: mock)
#   handy sim [test|full]     full stack on mock hardware (default: full)
#   handy real [test|full]    full stack on REAL Dynamixels (default: test) -- asks first
#   handy demo [speed]        run the Servo demo (default speed 0.5; use 0.3 on real arm)
#   handy jog <joint> <vel>   e.g. handy jog joint_2 -0.5
#   handy hand open|close|neutral|<rad> [sec]   gripper (joint_8 via hand_controller)
#   handy calibrate check|zero|limits   joint calibration (NO ROS running; see handy_calibrate)
#   handy motors <args>       raw motor CLI, e.g. handy motors scan / handy motors torque all off
#   handy status              show Servo status + controllers once
#   handy mode plan|servo     switch between MoveIt Plan&Execute and Servo
#   handy kill                stop all leftover ROS processes

HANDY_WS="${HANDY_WS:-$HOME/handy_ws}"
HANDY_PACKAGES="handy_description handy_motor_api handy_hardware handy_control handy_moveit_config handy_bringup handy_examples handy_calibrate"

_handy_confirm_real() {
  echo "!!  REAL HARDWARE: arm will be powered. Clear the area, hand on power."
  read -r -p "Type 'yes' to continue: " ok
  [ "$ok" = "yes" ] || { echo "Aborted."; return 1; }
}

handy() {
  local cmd="${1:-help}"; shift || true
  case "$cmd" in
    env)
      source /opt/ros/humble/setup.bash
      [ -f "$HANDY_WS/install/setup.bash" ] && source "$HANDY_WS/install/setup.bash"
      export PATH="$HOME/.local/bin:$PATH"
      echo "ROS $ROS_DISTRO + $HANDY_WS sourced"
      ;;
    build)
      ( cd "$HANDY_WS" || exit 1
        [ -d src/ivaHandy/ros ] && touch src/ivaHandy/ros/COLCON_IGNORE
        source /opt/ros/humble/setup.bash
        # --packages-up-to also builds dynamixel_sdk from third_party/
        # shellcheck disable=SC2086
        colcon build --symlink-install --packages-up-to $HANDY_PACKAGES "$@" )
      handy env
      ;;
    control)
      handy env >/dev/null
      case "${1:-mock}" in
        mock) ros2 launch handy_control control.launch.py use_mock:=true ;;
        real) _handy_confirm_real && ros2 launch handy_control control.launch.py use_mock:=false ;;
        *) echo "usage: handy control [mock|real]"; return 1 ;;
      esac
      ;;
    sim)
      handy env >/dev/null
      ros2 launch handy_bringup demo.launch.py use_mock:=true limits:="${1:-full}"
      ;;
    real)
      handy env >/dev/null
      _handy_confirm_real || return 1
      ros2 launch handy_bringup demo.launch.py use_mock:=false limits:="${1:-test}"
      ;;
    demo)
      handy env >/dev/null
      ros2 run handy_examples servo_demo --speed "${1:-0.5}"
      ;;
    jog)
      handy env >/dev/null
      [ $# -ge 2 ] || { echo "usage: handy jog <joint> <vel>"; return 1; }
      ros2 run handy_examples servo_jog joint "$1" "$2"
      ;;
    hand)
      handy env >/dev/null
      local pos
      case "$1" in
        open)    pos=-0.514 ;;
        close)   pos=0.125 ;;
        neutral) pos=0.0 ;;
        ''|*[!0-9.-]*) echo "usage: handy hand open|close|neutral|<rad -0.514..0.125> [seconds]"; return 1 ;;
        *)       pos="$1" ;;
      esac
      ros2 topic pub -w 1 --once /hand_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
        "{joint_names: [joint_8], points: [{positions: [$pos], time_from_start: {sec: ${2:-1}}}]}" >/dev/null \
        && echo "hand -> joint_8 = $pos (${2:-1} s)"
      ;;
    calibrate)
      handy env >/dev/null
      ros2 run handy_calibrate handy_calibrate "$@"
      ;;
    motors)
      handy env >/dev/null
      ros2 run handy_motor_api handy_motor_cli "$@"
      ;;
    status)
      handy env >/dev/null
      echo "== controllers =="; ros2 control list_controllers
      echo "== servo status (0 ok, 1/2 singularity, 3/4 collision, 5 joint limit) =="
      timeout 3 ros2 topic echo /servo_node/status --once 2>/dev/null | grep data || echo "no status (Servo not running?)"
      ;;
    mode)
      handy env >/dev/null
      case "$1" in
        plan)  ros2 control switch_controllers --deactivate servo_controller --activate arm_controller ;;
        servo) ros2 control switch_controllers --deactivate arm_controller --activate servo_controller ;;
        *) echo "usage: handy mode plan|servo"; return 1 ;;
      esac
      ;;
    kill)
      pkill -f ros2_control_node; pkill -f move_group; pkill -f rviz2
      pkill -f robot_state_publisher; pkill -f servo_node; pkill -f spawner
      pkill -f "ros2 service call"; ros2 daemon stop >/dev/null 2>&1
      sleep 1
      ps aux | grep -E "ros2_control|move_group|rviz2|servo_node|spawner" | grep -v grep \
        && echo "^ still running -- kill -9 <PID>" || echo "All ROS processes stopped."
      ;;
    *)
      sed -n '2,17p' "$HANDY_WS/src/ivaHandy/scripts/handy.sh" | sed 's/^# \{0,1\}//'
      ;;
  esac
}
