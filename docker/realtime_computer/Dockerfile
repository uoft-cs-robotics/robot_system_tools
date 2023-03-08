FROM osrf/ros:noetic-desktop-full

ENV WS_DIR="/root/git"
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
    python3-pip \    
    python3-distutils \
    gnuplot-qt\
    iproute2 \
    sshpass 
# Dependencies for Bash unit-tests
RUN apt-get update && apt-get install -y \
    bats \
    dialog \
    tmux \
    iputils-ping 

# Dependencies for franka-interface and protobuf
RUN apt-get update && apt-get install -y \
    git \
    vim \
    nano\
    wget \ 
    autoconf \ 
    automake \ 
    libtool \ 
    curl \ 
    make \
    g++ \
    unzip \
    python3-catkin-tools
# Build and install Protobuf 
RUN cd \
    # && mkdir git \
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
    ros-noetic-franka-gripper \
    ros-noetic-realsense2-camera

RUN cd ~/git \ 
    && git clone --recurse-submodules https://github.com/iamlab-cmu/franka-interface \
    && cd franka-interface 

RUN echo "cloning libfranka"
RUN cd ~/git/franka-interface \
    && bash ./bash_scripts/clone_libfranka.sh 5

RUN echo "building libfranka"
RUN cd ~/git/franka-interface \
    && bash ./bash_scripts/make_libfranka.sh

RUN echo "building franka-interface"
RUN cd ~/git/franka-interface \
    && bash ./bash_scripts/make_franka_interface.sh

RUN echo "making ROS catkin workspace for franka-interface"
RUN cd ~/git/franka-interface \
    && source /opt/ros/noetic/setup.bash \
    && bash ./bash_scripts/make_catkin.sh 

RUN echo "cloning and building libzmq"
RUN cd ~/git \ 
    && git clone https://github.com/zeromq/libzmq  \
    && cd libzmq \ 
    && mkdir build \ 
    && cd build \ 
    && cmake .. \ 
    && sudo make  install 

RUN echo "cloning and building cppzmq"
RUN cd ~/git \ 
    && git clone https://github.com/zeromq/cppzmq \
    && cd cppzmq \ 
    && mkdir build \ 
    && cd build \ 
    && cmake .. \ 
    && sudo make  install 

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source ~/git/franka-interface/catkin_ws/devel/setup.bash " >> ~/.bashrc
RUN pip3 install zmq pyrealsense2 numba scipy   

ARG DEBIAN_FRONTEND=dialog

