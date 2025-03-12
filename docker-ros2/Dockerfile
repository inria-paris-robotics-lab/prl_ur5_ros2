################################
# Base image
################################

FROM ubuntu:24.04 AS base

# Here you can choose your ros distro
ENV ROS_DISTRO=jazzy

ARG DOCKERGRP=999

# Prevents any interaction during installation by selecting the default response
ENV DEBIAN_FRONTEND=noninteractive

# Check language (compatible ros2)
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt-get/lists/*
ENV LANG=en_US.UTF-8
ENV TERM=xterm-256color
ENV SHELL=/bin/bash

RUN apt-get -y update && apt-get -y upgrade 

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt-get/lists/*

# Install requirements
RUN apt-get update && apt-get install -y --no-install-recommends\
    curl \
    software-properties-common \
    sudo \
    wget \
    lsb-release gnupg \
    && rm -rf /var/lib/apt-get/lists/*

# Install ROS2
RUN sudo add-apt-repository universe
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get -y upgrade \
  python3-argcomplete \
  && apt-get install -y --no-install-recommends \
   ros-${ROS_DISTRO}-desktop \
   ros-dev-tools \
   ros-${ROS_DISTRO}-ros2-control \
   ros-${ROS_DISTRO}-ros2-controllers \
   ros-${ROS_DISTRO}-ur \
   ros-${ROS_DISTRO}-librealsense2* \
   ros-${ROS_DISTRO}-realsense2-* \
   ros-${ROS_DISTRO}-moveit \
  && rm -rf /var/lib/apt-get/lists/*


ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu:/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/local/lib/python3.10/dist-packages:/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages

#Source ros
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN rosdep init \
  &&rosdep update 


################################
#  UR ros driver
################################

FROM base AS ros2_complete

RUN sudo apt-get install -y ros-${ROS_DISTRO}-ur


################################
# Complete image with gazebo 
################################

FROM ros2_complete AS full
ENV DEBIAN_FRONTEND=noninteractive

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control\
  && rm -rf /var/lib/apt/lists/*



################################
#  Dev tools and non root user
################################

FROM full AS final
ENV DEBIAN_FRONTEND=noninteractive

# Install xauth for gui applications
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    xauth \
    && rm -rf /var/lib/apt-get/lists/*

# Install docker 
RUN sudo apt-get update && apt-get install -y ca-certificates curl \
  && sudo install -m 0755 -d /etc/apt/keyrings \
  && sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc \
  && sudo chmod a+r /etc/apt/keyrings/docker.asc \
  && echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null \
  && sudo apt-get update

RUN sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# RUN sudo groupmod -g ${DOCKERGRP} docker 

# Install dev tools
RUN apt-get update && apt-get install -y --no-install-recommends\
  byobu \
  bash-completion \
  net-tools \
  nano \
  && rm -rf /var/lib/apt-get/lists/*

#byobu disable by default 
RUN byobu-enable 

# Create non root user matching host user
ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Create new user and his home directory
RUN deluser --remove-home `id -nu ${USER_GID}` || true \
 ; groupadd --gid $USER_GID $USERNAME \
 && useradd --uid ${USER_GID} --gid ${USER_GID} --create-home ${USERNAME} \
 # support du sudo par l'user
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${USER_GID}:${USER_GID} /home/${USERNAME} 

RUN sudo usermod -aG docker $USERNAME


# Set default user ros
USER ${USERNAME}
# RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-circleci" >> /home/${USERNAME}/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash " >> /home/${USERNAME}/.bashrc
RUN echo 'export PS1="\[\e]0;\u@$ROS_DISTRO-container: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;34m\]\u@$ROS_DISTRO-container\[\033[00m\]:\[\033[01;35m\]\w\[\033[00m\]\$ "' >> /home/${USERNAME}/.bashrc
# Set default working directory
WORKDIR /home/${USERNAME}

# Create a directory to store data
RUN mkdir share
