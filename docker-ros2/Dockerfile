################################
# Base image
################################

FROM ubuntu:24.04 AS base

# Here you can choose your ros distro
ENV ROS_DISTRO=jazzy

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
  && apt-get install -y --no-install-recommends \
  python3-argcomplete \
  ros-${ROS_DISTRO}-desktop \
  ros-dev-tools \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-ur \
  ros-${ROS_DISTRO}-librealsense2* \
  ros-${ROS_DISTRO}-realsense2-* \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-py \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-image-pipeline \
  && rm -rf /var/lib/apt-get/lists/*
  
# Orbbec driver dependencies

RUN sudo apt-get update && apt-get install -y \
  libgflags-dev nlohmann-json3-dev \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-image-publisher \
  ros-${ROS_DISTRO}-camera-info-manager \
  ros-${ROS_DISTRO}-diagnostic-updater \
  ros-${ROS_DISTRO}-diagnostic-msgs \
  ros-${ROS_DISTRO}-statistics-msgs \
  ros-${ROS_DISTRO}-backward-ros libdw-dev \
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
# Complete ros2 image with gazebo 
################################

FROM base AS ros2_complete
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

FROM ros2_complete AS full
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

# Install dev tools
RUN apt-get update && apt-get install -y --no-install-recommends\
  byobu \
  bash-completion \
  net-tools \
  nano \
  vim \
  git \
  tmux \
  python3-vcstool \
  && rm -rf /var/lib/apt-get/lists/*

# Install Pinocchio 
# Installer les dépendances pour la gestion des dépôts
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
# Créer le répertoire pour les clés
 && mkdir -p /etc/apt/keyrings \
# Télécharger la clé GPG, la convertir et la stocker au bon endroit
 && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | gpg --dearmor -o /etc/apt/keyrings/robotpkg.gpg \
# Ajouter le dépôt en spécifiant la clé de signature
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/robotpkg.gpg] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" > /etc/apt/sources.list.d/robotpkg.list \
# Mettre à jour les sources et installer le paquet
 && apt-get update \
 && apt-get install -y robotpkg-py3*-pinocchio \
# Nettoyer le cache apt pour réduire la taille de l'image
 && rm -rf /var/lib/apt/lists/*


#byobu disable by default 
RUN byobu-enable 

# Create non root user matching host user
ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=1001
ARG INPUT_GRP=106


RUN if getent group ${USER_GID} >/dev/null 2>&1; then \
        EXISTING_GROUP_NAME=$(getent group ${USER_GID} | cut -d: -f1); \
        echo "Deleting conflicting group ${EXISTING_GROUP_NAME} with GID ${USER_GID}"; \
        delgroup ${EXISTING_GROUP_NAME}; \
    fi \
 && if getent passwd ${USER_UID} >/dev/null 2>&1; then \
        EXISTING_USER_NAME=$(getent passwd ${USER_UID} | cut -d: -f1); \
        echo "Deleting conflicting user ${EXISTING_USER_NAME} with UID ${USER_UID}"; \
        deluser --remove-home ${EXISTING_USER_NAME}; \
    fi 

# Create new user and his home directory
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID --create-home --shell /bin/bash $USERNAME \
  && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && (getent group docker || groupadd docker) \
  && usermod -aG docker $USERNAME \
  && usermod -aG video $USERNAME \
  && groupadd -g $INPUT_GRP input \
  && usermod -aG input $USERNAME \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash " >> /home/${USERNAME}/.bashrc \
  && echo 'export PS1="\[\e]0;\u@$ROS_DISTRO-container: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;34m\]\u@$ROS_DISTRO-container\[\033[00m\]:\[\033[01;35m\]\w\[\033[00m\]\$ "' >> /home/${USERNAME}/.bashrc \
  && echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/${USERNAME}/.bashrc 


# Config Tmux
COPY --chown=${USERNAME}:${USERNAME} utils/tmux.conf /home/${USERNAME}/.tmux.conf

## ADD pinocchio 
RUN echo "export PATH=/opt/openrobots/bin:\$PATH" >> /home/${USERNAME}/.bashrc \
 && echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH" >> /home/${USERNAME}/.bashrc \
 && echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH" >> /home/${USERNAME}/.bashrc \
 && echo "export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:\$PYTHONPATH" >> /home/${USERNAME}/.bashrc \
 && echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >> /home/${USERNAME}/.bashrc

# Set default user ros
USER ${USERNAME}
# Set default working directory
WORKDIR /home/${USERNAME}
# Create a directory to store data
RUN mkdir share

################################
# Orbbec driver
SHELL ["/bin/bash", "-c"]

RUN mkdir -p ~/ros2_utils_ws/src && \
    cd ~/ros2_utils_ws/src && \
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git -b v2-main && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master && \
    # Go to the workspace root
    cd ~/ros2_utils_ws && \
    # Source the ROS 2 environment directly
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    # Now, the environment is correctly loaded for the build command.
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- UPDATE .bashrc ---
# Sourcing the new workspace for future interactive shells
RUN echo "source ~/ros2_utils_ws/install/setup.bash" >> ~/.bashrc
