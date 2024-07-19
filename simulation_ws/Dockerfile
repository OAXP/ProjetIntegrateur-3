FROM ros:humble-ros-core-jammy

# replace shell with bash so we can source files
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

WORKDIR /app

#install ros2 requirements
RUN apt update; apt upgrade -y
RUN apt install lsb-release wget curl gnupg python3-pip git -y

RUN apt install software-properties-common -y
RUN pip3 install vcstool colcon-common-extensions
RUN add-apt-repository universe

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# installing gazebo requirements
RUN apt-get update && apt install mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev -y

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt update; apt install ignition-fortress -y

RUN echo "export GZ_VERSION=fortress" >> /root/.bashrc

COPY . .

RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble && \
    colcon build

RUN pip install playsound

ENV DISPLAY=:0

CMD ["/bin/bash", "-c", "source install/setup.bash && ros2 launch simulation_bringup diff_drive.launch.py"]