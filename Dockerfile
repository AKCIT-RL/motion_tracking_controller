# Dockerfile para motion_tracking_controller
# Baseado em ros:jazzy que já contém ROS 2 Jazzy Desktop instalado

FROM ros:jazzy

# Evitar prompts interativos durante instalação
ENV DEBIAN_FRONTEND=noninteractive

# 1. Instalar dependências do sistema e ferramentas de build
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

# 2. Configurar rosdep
RUN rosdep init || true
RUN rosdep update

# 3. Adicionar repositório legged_buildfarm (para legged_control2)
RUN echo "deb [trusted=yes] https://github.com/qiayuanl/legged_buildfarm/raw/noble-jazzy-amd64/ ./" | tee /etc/apt/sources.list.d/qiayuanl_legged_buildfarm.list
RUN echo "yaml https://github.com/qiayuanl/legged_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_legged_buildfarm.list

# 4. Adicionar repositório unitree_buildfarm (para pacotes Unitree específicos)
RUN echo "deb [trusted=yes] https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/ ./" | tee /etc/apt/sources.list.d/qiayuanl_unitree_buildfarm.list
RUN echo "yaml https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | tee /etc/ros/rosdep/sources.list.d/2-qiayuanl_unitree_buildfarm.list

# 5. Adicionar repositório simulation_buildfarm (para MuJoCo - opcional, mas necessário para simulação)
RUN echo "deb [trusted=yes] https://github.com/qiayuanl/simulation_buildfarm/raw/noble-jazzy-amd64/ ./" | tee /etc/apt/sources.list.d/qiayuanl_simulation_buildfarm.list
RUN echo "yaml https://github.com/qiayuanl/simulation_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | tee /etc/ros/rosdep/sources.list.d/3-qiayuanl_simulation_buildfarm.list

# 6. Atualizar apt e instalar pacotes
RUN apt-get update && apt-get install -y \
    ros-jazzy-legged-control-base \
    ros-jazzy-unitree-description \
    ros-jazzy-unitree-systems \
    ros-jazzy-mujoco-ros2-control \
    && rm -rf /var/lib/apt/lists/*


# 7. Criar workspace ROS 2
WORKDIR /workspace
RUN mkdir -p colcon_ws/src

# 8. Clonar repositórios necessários
WORKDIR /workspace/colcon_ws/src
RUN git clone https://github.com/qiayuanl/unitree_bringup.git
# O motion_tracking_controller será copiado do contexto do build
COPY . motion_tracking_controller

# 9. Instalar dependências via rosdep
WORKDIR /workspace/colcon_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# 10. Compilar workspace
# Source ROS 2 setup antes de compilar
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo --packages-up-to unitree_bringup"

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo --packages-up-to motion_tracking_controller"

# 11. Criar script de entrypoint
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /entrypoint.sh && \
    echo 'if [ -f /workspace/colcon_ws/install/setup.bash ]; then' >> /entrypoint.sh && \
    echo '    source /workspace/colcon_ws/install/setup.bash' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

# 12. Configurar diretório de trabalho
WORKDIR /workspace/colcon_ws

# 13. Definir entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

