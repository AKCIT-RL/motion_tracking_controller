# Dockerfile for motion_tracking_controller
# Based on ros:jazzy which already contains ROS 2 Jazzy Desktop installed

FROM ros:jazzy

# Avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install system dependencies and build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Configure rosdep
RUN rosdep init || true
RUN rosdep update

# 3. Add legged_buildfarm repository (for legged_control2)
RUN echo "deb [trusted=yes] https://github.com/qiayuanl/legged_buildfarm/raw/noble-jazzy-amd64/ ./" | tee /etc/apt/sources.list.d/qiayuanl_legged_buildfarm.list
RUN echo "yaml https://github.com/qiayuanl/legged_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_legged_buildfarm.list

# 4. Add unitree_buildfarm repository (for Unitree-specific packages)
RUN echo "deb [trusted=yes] https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/ ./" | tee /etc/apt/sources.list.d/qiayuanl_unitree_buildfarm.list
RUN echo "yaml https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | tee /etc/ros/rosdep/sources.list.d/2-qiayuanl_unitree_buildfarm.list

# 5. Add simulation_buildfarm repository (for MuJoCo - optional but required for simulation)
RUN echo "deb [trusted=yes] https://github.com/qiayuanl/simulation_buildfarm/raw/noble-jazzy-amd64/ ./" | tee /etc/apt/sources.list.d/qiayuanl_simulation_buildfarm.list
RUN echo "yaml https://github.com/qiayuanl/simulation_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | tee /etc/ros/rosdep/sources.list.d/3-qiayuanl_simulation_buildfarm.list

# 6. Update apt and install packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-legged-control-base \
    ros-jazzy-unitree-description \
    ros-jazzy-unitree-systems \
    ros-jazzy-mujoco-ros2-control \
    && rm -rf /var/lib/apt/lists/*


# 7. Create ROS 2 workspace
WORKDIR /workspace
RUN mkdir -p colcon_ws/src

# 8. Clone necessary repositories
WORKDIR /workspace/colcon_ws/src
RUN git clone https://github.com/qiayuanl/unitree_bringup.git
# motion_tracking_controller will be copied from build context
COPY . motion_tracking_controller

# 9. Install dependencies via rosdep
WORKDIR /workspace/colcon_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# 10. Build workspace
# Source ROS 2 setup before building
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo --packages-up-to unitree_bringup"

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo --packages-up-to motion_tracking_controller"

# 11. Create entrypoint script
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /entrypoint.sh && \
    echo 'if [ -f /workspace/colcon_ws/install/setup.bash ]; then' >> /entrypoint.sh && \
    echo '    source /workspace/colcon_ws/install/setup.bash' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

# 12. Set working directory
WORKDIR /workspace/colcon_ws

# 13. Define entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

