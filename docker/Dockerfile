# build a image for developing in ubuntu 20.04
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04 
ENV TZ=America/New_York
ARG DEBIAN_FRONTEND=noninteractive
# install some necessary tools
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    git \
    curl \
    wget \
    lsb-release \
    sudo \
    gnupg2 \
    build-essential \
    libglpk-dev \
    libglvnd0\
    mesa-utils \
    cmake \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    terminator \
    libnlopt-dev \
    locales && \
    ros-noetic-map-server \
    locale-gen en_US.UTF-8\
    && rm -rf /var/lib/apt/lists/*

# ENV LD_LIBRARY_PATH=/usr/local/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH:$LD_LIBRARY_PATH
ENV NVIDIA_VISIBLE_DEVICES=all    
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ROS 1 

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-pip \
    ros-noetic-map-server \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update


ENV ROS_DISTRO=noetic
ENV ROS_PACKAGE_PATH=/opt/ros/$ROS_DISTRO/share
ENV PATH=/opt/ros/$ROS_DISTRO/bin:$PATH
ENV PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3/dist-packages
ENV LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:$LD_LIBRARY_PATH


# RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" > /home/${user}/.bashrc \
#     && chown ${user}:${group} /home/${user}/.bashrc

# copt
ENV COPT_HOME=/opt/copt72
ENV COPT_LICENSE_DIR=/opt/copt72
ENV PATH=$COPT_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$COPT_HOME/lib:$LD_LIBRARY_PATH

# change it to your own user name and make sure it is consistent with the machine name bundled with your copt license
ARG user=yulin
ARG group=yulin
ARG uid=1000
ARG gid=1000
ARG home=/home/${user}
RUN mkdir -p /etc/sudoers.d \
    && groupadd -g ${gid} ${group} \
    && useradd -d ${home} -u ${uid} -g ${gid} -m -s /bin/bash ${user} \
    && echo "${user} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sudoers_${user}
USER ${user}
RUN sudo usermod -a -G video ${user}
COPY copt72 /opt/copt72
WORKDIR ${home}

CMD ["terminator"]

# install ALTRO
# RUN git clone https://github.com/bjack205/altro \
#     && cd altro \
#     && mkdir build && cd build \
#     && cmake -D BUILD_SHARED_LIBS=ON ..\
#     && cmake --build . -j4 \
#     && sudo cmake --install .

# # install frtree planner
# RUN https://github.com/YulinLi0/navigation_with_tree_of_free_regions.git \
#     && cd navigation_with_tree_of_free_regions \
#     && catkin build pop_planner \
#     && source devel/setup.bash

# !set up your robot, sensor, and launch the planner
# roslaunch pop_planner pop_planner.launch


# and open shell
# CMD ["/bin/bash"]



