# UR10e + OnRobot RG6 - ROS 2 Humble runtime image
# Builds the workspace from /workspace/src (mounted from your host clone).

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive     ROS_DISTRO=humble     PYTHONUNBUFFERED=1

# ---- System deps ----
RUN apt-get update && apt-get install -y --no-install-recommends         python3-colcon-common-extensions         python3-rosdep         python3-vcstool         python3-pip         git         build-essential         cmake         ros-humble-ur         ros-humble-ur-robot-driver         ros-humble-ur-moveit-config         ros-humble-ur-description         ros-humble-controller-manager         ros-humble-joint-state-publisher-gui         ros-humble-joint-state-broadcaster         ros-humble-joint-trajectory-controller         ros-humble-position-controllers         ros-humble-velocity-controllers         ros-humble-rosbridge-suite         ros-humble-rviz2         ros-humble-xacro         ros-humble-tf2-ros         ros-humble-moveit         libxcb-cursor0         libxkbcommon-x11-0         x11-apps     && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pymodbus==2.5.3

# Initialize rosdep (ignore failure if already initialized)
RUN (rosdep init 2>/dev/null || true) && rosdep update

WORKDIR /workspace

# Copy entrypoint
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# rosbridge default port
EXPOSE 9090

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
