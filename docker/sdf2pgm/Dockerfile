FROM osrf/ros:humble-desktop

# Instalações básicas
RUN apt-get update && apt-get install -y \
    git \
    wget \
    lsb-release \
    sudo \
    curl \
    gnupg2 \
    lsb-release \
    libboost-dev \
    libpcl-dev \
    x11-apps \
    xdg-utils \
    && rm -rf /var/lib/apt/lists/*

# Configurações do ambiente
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1

# Criação do workspace
RUN mkdir -p /root/robot_ws/src
WORKDIR /root/robot_ws/src

# Clonagem do repositório
RUN git clone https://github.com/arshadlab/gazebo_map_creator.git

# Instalação de dependências ROS
WORKDIR /root/robot_ws
RUN apt-get update && apt-get install -y python3-colcon-common-extensions libpcap-dev
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Compilação do workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Fonte do ambiente
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/robot_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
