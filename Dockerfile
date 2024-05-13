# Ubuntu 22.04
FROM ros:humble-ros-base-jammy

# Non-root user access
ARG USERNAME=uzer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime
  
# Tools necessary and useful during development
RUN apt update && \
 DEBIAN_FRONTEND=noninteractive \
 apt install --no-install-recommends -y \
        build-essential \
        atop \
	ca-certificates \
        cmake \
        cppcheck \
	curl \
        expect \
        gdb \
        git \
	gnupg2 \
        gnutls-bin \
	iputils-ping \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
	libeigen3-dev \
        libfcl-dev \
        libgflags-dev \
        libgles2-mesa-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
	lsb-release \
	net-tools \
	pkg-config \
        protobuf-compiler \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
	ruby \
        software-properties-common \
	sudo \
        vim \
	wget \
	xvfb \
 && apt clean -qq

# Setup locale
RUN sudo apt update && sudo apt install locales \
  && sudo locale-gen en_US en_US.UTF-8 \
  && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ARG ROSDIST=humble
ARG GZDIST=garden
ENV GZ_VERSION garden

# Set up repo to install Ignition
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'
  
# Install Gazebo and ROS2 Desktop
RUN apt update \
  && apt install -y --no-install-recommends \
     gz-${GZDIST} \
     ros-${ROSDIST}-desktop \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean -qq

# Install some 'standard' ROS packages and utilities.
RUN apt update \
  && apt install -y --no-install-recommends \
     python3-colcon-common-extensions \
     python3-vcstool \
     python3-sdformat13 \
     ros-${ROSDIST}-actuator-msgs \
     ros-${ROSDIST}-ament-cmake-pycodestyle \
     ros-${ROSDIST}-image-transport \
     ros-${ROSDIST}-image-transport-plugins \
     ros-${ROSDIST}-joy-teleop \
     ros-${ROSDIST}-joy-linux \
     ros-${ROSDIST}-mavros-msgs \
     ros-${ROSDIST}-radar-msgs \
     ros-${ROSDIST}-ros-gzgarden \
     ros-${ROSDIST}-rqt-graph \
     ros-${ROSDIST}-rqt-image-view \
     ros-${ROSDIST}-rqt-plot \
     ros-${ROSDIST}-rqt-topic \
     ros-${ROSDIST}-rviz2 \
     ros-${ROSDIST}-xacro \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt clean -qq

USER $USERNAME

# Define an argument for the parent directory of the workspace
ARG WORKSPACE_DIR=/home/$USERNAME/rtu_usv_ws
# Define an argument for the src directory within the workspace
ARG WORKSPACE_SRC_DIR=$WORKSPACE_DIR/src

# Create the workspace src directory
RUN mkdir -p $WORKSPACE_SRC_DIR

# Copy package directories into the workspace src
COPY vrx_gz $WORKSPACE_SRC_DIR/vrx_gz
COPY vrx_ros $WORKSPACE_SRC_DIR/vrx_ros
COPY vrx_urdf $WORKSPACE_SRC_DIR/vrx_urdf

# Build the workspace
RUN cd $WORKSPACE_DIR && \
    /bin/bash -c "source /opt/ros/$ROSDIST/setup.bash && colcon build --merge-install"

# Automatically source the ROS installation and the workspace in every new shell
RUN echo "source /opt/ros/$ROSDIST/setup.sh" >> "/home/$USERNAME/.bashrc"
RUN echo "source $WORKSPACE_DIR/install/setup.bash" >> "/home/$USERNAME/.bashrc"

COPY requirements.txt requirements.txt
RUN pip install -r requirements.txt