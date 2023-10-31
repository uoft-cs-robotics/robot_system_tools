ARG image=nvidia/cudagl:11.3.0-devel-ubuntu20.04
FROM ${image}

ARG DEBIAN_FRONTEND=noninteractive
ARG workstation_ip=192.168.0.1
ARG use_robotiq=0

ENV WS_DIR="/root/git"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

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
    sshpass \
    iproute2 \
    python3-pip \
    gnome-terminal\
    nano\
    vim
# Dependencies for Bash unit-tests
RUN apt-get update && apt-get install -y \
    bats \
    dialog \
    tmux \
    iputils-ping 	

# ROS Install
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-catkin-tools \
    ros-noetic-jsk-tools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO noetic

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
    # && mkdir git \
    && cd git \ 
    && wget https://github.com/protocolbuffers/protobuf/releases/download/v3.11.4/protobuf-all-3.11.4.zip \ 
    && unzip protobuf-all-3.11.4.zip \ 
    && cd protobuf-3.11.4 \ 
    && ./configure \
    && make -j$(nproc) \
    && make check -j$(nproc) \ 
    && sudo make install \ 
    && sudo ldconfig \
    && rm /root/git/protobuf-all-3.11.4.zip

# Dependencies for franka-interface and protobuf
RUN apt-get install -y \
    ros-noetic-libfranka \
    ros-noetic-franka-ros \
    ros-noetic-franka-gripper \
    ros-noetic-realsense2-camera

RUN echo "cloning building autolab perception"
RUN cd ~/git \ 
    && git clone --recurse-submodules https://github.com/BerkeleyAutomation/perception
RUN cd ~/git/perception \
    && sudo pip3 install -e .

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
 
# RUN echo "export ROS_HOSTNAME=192.168.0.1" >> ~/.bashrc
# RUN echo "export ROS_IP=192.168.0.1" >> ~/.bashrc
# RUN echo "export ROS_MASTER_URI=http://192.168.0.1:11311" >> ~/.bashrc

# RUN echo "export ROS_HOSTNAME=${workstation_ip}" >> ~/.bashrc
RUN echo "export ROS_IP=${workstation_ip}" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://${workstation_ip}:11311" >> ~/.bashrc

RUN sudo pip3 install --upgrade protobuf

RUN sudo pip3 install numpy --force-reinstall
#reinstalling numba seems to solve an error when we import FrankaArm from frankapy
RUN pip3 install zmq pyrealsense2 numba scipy open3d   wrapt_timeout_decorator


# install librealsense sdk
# RUN mkdir -p /etc/apt/keyrings \ 
#     && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
#     sudo tee /etc/apt/sources.list.d/librealsense.list \
#     && apt-get update \ 
#     && apt-get install librealsense2-dkms \ 
#     && apt-get install librealsense2-utils

# RUN cd /root/git/franka_arm_infra/franka_control_suite \
#     && mkdir build \
#     && cd build \ 
#     && cmake .. \
#     && make

RUN if [[ "x$use_robotiq" == "x1" ]] ;\
    then echo "use_robotiq is set to TRUE" \
    && apt-get install ros-noetic-soem \
    && pip3 install -U pymodbus==2.5.3 \
    && cd ~/git \
    && mkdir -p robotiq_ws/src \
    && cd robotiq_ws/src \
    && git clone https://github.com/pairlab/robotiq.git \
    && cd ../ \ 
    && source /opt/ros/noetic/setup.bash\
    && catkin build \
    && echo "source ~/git/robotiq_ws/devel/setup.bash" >> ~/.bashrc ; \
    fi
