# ROS 2 Jazzy + Gazebo Harmonic + Nav2 + SLAM Toolbox + TurtleBot3 (Ubuntu 24.04)
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-twist-mux \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-scipy \
        python3-pil \
        python3-pytest \
        python3-colcon-common-extensions \
        git make \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3 simulation stack, built from source (jazzy branches) as the README specifies
RUN mkdir -p /opt/turtlebot3_ws/src && cd /opt/turtlebot3_ws/src \
    && git clone --depth 1 -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && git clone --depth 1 -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git \
    && git clone --depth 1 -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    && apt-get update \
    && (rosdep update --rosdistro jazzy || true) \
    && source /opt/ros/jazzy/setup.bash \
    && rosdep install --from-paths . --ignore-src -y --rosdistro jazzy \
         --skip-keys "turtlebot3_manipulation_gazebo" || true \
    && rm -rf /var/lib/apt/lists/* \
    && cd /opt/turtlebot3_ws \
    && colcon build --packages-up-to turtlebot3_gazebo turtlebot3_msgs \
    && rm -rf build log

ENV TURTLEBOT3_MODEL=waffle \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/turtlebot3_ws/install/setup.bash' >> /root/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
