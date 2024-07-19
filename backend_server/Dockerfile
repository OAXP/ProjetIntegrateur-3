FROM ros:humble-ros-core-jammy

# replace shell with bash so we can source files
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

WORKDIR /app

COPY ./backend_server/package*json ./

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

ENV NODE_VERSION=18.19.0
ENV NVM_DIR /usr/local/nvm

RUN mkdir -p /usr/local/nvm

# install nodejs v18 and npm install
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash && \
    source $NVM_DIR/nvm.sh && \
    nvm install $NODE_VERSION && \
    nvm use $NODE_VERSION && \
    source /opt/ros/$ROS_DISTRO/setup.sh && \
    npm install

ENV NODE_PATH=$NVM_DIR/versions/node/v$NODE_VERSION/lib/node_modules
ENV PATH=$NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH

RUN ls $NVM_DIR/versions/node && source $NVM_DIR/nvm.sh && nvm use $NODE_VERSION && node -v && npm -v

# build ros2 interfaces
RUN mkdir -p ros_ws/src/communication_interfaces

COPY simulation_ws/src/communication_interfaces ./ros_ws/src/communication_interfaces

WORKDIR /app/ros_ws

RUN source $NVM_DIR/nvm.sh && \
    source /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build && \
    source install/setup.sh && \
    cd /app && \
    npx generate-ros-messages

WORKDIR /app

# copy backend source code
COPY ./backend_server .

ENV PORT=3000

EXPOSE 3000

CMD ["/bin/bash", "-c", "source ros_ws/install/setup.bash && npm start"]
