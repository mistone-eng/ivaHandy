# Handy development container: ROS 2 Humble desktop (MATE) with VNC/noVNC, MoveIt 2, MoveIt Servo,
# ros2_control and everything the ivaHandy + ivaDynamixel packages need.
#
# Base image: https://github.com/Tiryoh/docker-ros2-desktop-vnc (multi-arch: arm64 Macs, amd64 PCs).
# For a reproducible build, pin it by digest: `docker images --digests tiryoh/ros2-desktop-vnc`.
#
# The workspace (~/handy_ws) is bind-mounted from the host and built inside the container with
# `handy build`, so this image only holds dependencies. See docs/docker.md.
FROM tiryoh/ros2-desktop-vnc:humble

ENV DEBIAN_FRONTEND=noninteractive

# Packages installed by hand in the reference container (2026-11): MoveIt 2 + Servo, ros2_control,
# xacro, the Robotis DynamixelSDK (ivaDynamixel), the joint slider GUI (display.launch.py),
# plus build tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-moveit \
        ros-humble-moveit-servo \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-xacro \
        ros-humble-dynamixel-sdk \
        ros-humble-joint-state-publisher-gui \
        python3-colcon-common-extensions \
        python3-rosdep \
        git \
        curl \
    && rm -rf /var/lib/apt/lists/*

# GitHub CLI from GitHub's own apt repo (Ubuntu's gh 2.4 is too old: `gh pr edit` fails).
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
        -o /usr/share/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
        > /etc/apt/sources.list.d/github-cli.list \
    && apt-get update && apt-get install -y --no-install-recommends gh \
    && rm -rf /var/lib/apt/lists/*

# Everything else the packages declare (build, exec and test dependencies, incl. the ament linters),
# resolved from their package.xml files so the image follows the code. Only the manifests are
# needed; third_party/ivaDynamixel must be checked out (git submodule update --init).
COPY ros2 /tmp/handy_src/ros2
COPY testing /tmp/handy_src/testing
COPY third_party /tmp/handy_src/third_party
RUN (rosdep init || true) \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths /tmp/handy_src --ignore-src -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/* /tmp/handy_src

# Shell setup for every user (the base image creates the user at container start, and an existing
# home volume skips /etc/skel): ROS, colcon completion, ~/.local/bin (Claude Code), the `handy` helper.
RUN { \
      echo ''; \
      echo '# --- Handy ---'; \
      echo 'source /opt/ros/humble/setup.bash'; \
      echo '[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash'; \
      echo 'export PATH="$HOME/.local/bin:$PATH"'; \
      echo '[ -f "$HOME/handy_ws/src/ivaHandy/scripts/handy.sh" ] && source "$HOME/handy_ws/src/ivaHandy/scripts/handy.sh"'; \
    } >> /etc/bash.bashrc

# Keep the base image's ENTRYPOINT/CMD (creates the user, starts VNC + noVNC via supervisord).
