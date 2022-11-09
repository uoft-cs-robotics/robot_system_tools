#FROM ubuntu:focal

FROM osrf/ros:noetic-desktop-full


ENV WS_DIR="/root"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

# Dependencies for latency plot
RUN apt-get update && apt-get install -y \
    curl \
    grep \
    build-essential \
    make \
    libnuma-dev \
    python3 \
    python3-distutils \
    gnuplot-qt \ 
    sshpass 
# Dependencies for Bash unit-tests
RUN apt-get update && apt-get install -y \
    bats \
    dialog \
    tmux \
    iputils-ping 	

# Fetch most recent version of rt-tests and mklatencyplot
# Dependencies for latency plot
RUN apt-get update && apt-get install -y \
    curl \
    grep \
    build-essential \
    make \
    libnuma-dev \
    python3 \
    python3-distutils \
    gnuplot-qt\
    iproute2 \
    python3-pip \
    gnome-terminal
# Dependencies for Bash unit-tests
RUN apt-get update && apt-get install -y \
    bats \
    dialog \
    tmux \
    iputils-ping 

# Dependencies for franka-interface and protobuf
RUN apt-get update && apt-get install -y \
    git \
    wget \ 
    autoconf \ 
    automake \ 
    libtool \ 
    curl \ 
    make \
    g++ \
    unzip \
    python3-catkin-tools \ 
    python3-tk
# Build and install Protobuf 
RUN cd \
    && mkdir git \
    && cd git \ 
    && wget https://github.com/protocolbuffers/protobuf/releases/download/v3.11.4/protobuf-all-3.11.4.zip \ 
    && unzip protobuf-all-3.11.4.zip \ 
    && cd protobuf-3.11.4 \ 
    && ./configure \
    && make -j$(nproc) \
    && make check -j$(nproc) \ 
    && sudo make install \ 
    && sudo ldconfig 

# Dependencies for franka-interface and protobuf
RUN apt-get install -y \
    ros-noetic-libfranka \
    ros-noetic-franka-ros \
    ros-noetic-franka-gripper

RUN echo "cloning frankapy"

RUN cd ~/git \ 
    && git clone --recurse-submodules https://github.com/Ruthrash/frankapy 



RUN echo "building frankapy python modules"
RUN cd ~/git/frankapy \
    && sudo pip3 install -e .

RUN echo "compiling catkin_ws"
RUN cd ~/git/frankapy \ 
    && source /opt/ros/noetic/setup.bash \
    && ./bash_scripts/make_catkin.sh

RUN apt-get install -y \
    ros-noetic-libfranka \
    ros-noetic-franka-ros \
    ros-noetic-franka-gripper


RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source ~/git/frankapy/catkin_ws/devel/setup.bash " >> ~/.bashrc
 
RUN echo "export ROS_HOSTNAME=192.168.0.1" >> ~/.bashrc
RUN echo "export ROS_IP=192.168.0.1" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://192.168.0.1:11311" >> ~/.bashrc

RUN sudo pip3 install --upgrade protobuf

RUN sudo pip3 install numpy --force-reinstall

RUN cd /root/git/franka_arm_infra/franka_control_suite \
    && mkdir build \
    && cd build \ 
    && cmake .. \
    && make
ARG DEBIAN_FRONTEND=dialog

