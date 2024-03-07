ARG image=nvidia/cudagl:11.3.0-devel-ubuntu20.04
FROM ${image}

ARG DEBIAN_FRONTEND=noninteractive
ARG workstation_ip=192.168.0.1
ARG use_robotiq=1
ARG build_dope=1
ARG build_contactgraspnet=1
ARG build_megapose=1
ENV WS_DIR="/root/git"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]
# Dependencies for latency plot
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
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

RUN echo "Installing conda"
ENV PATH /opt/conda/bin:$PATH
RUN  export CONDA_DIR=/opt/conda \
    && wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh  \
    && /bin/bash ~/miniconda.sh -b -p /opt/conda \ 
    && export PATH=$CONDA_DIR/bin:$PATH \ 
    && source ~/.bashrc \
    && conda config --set auto_activate_base false \ 
    && conda init bash \ 
    && conda install python=3.8
    # && conda deactivate

RUN echo "cloning building autolab perception"
RUN cd ~/git \ 
    && git clone --recurse-submodules https://github.com/BerkeleyAutomation/perception
RUN cd ~/git/perception \
    && pip3 install -e .

RUN pip3 install -U rosdep rosinstall_generator wstool rosinstall six vcstools

RUN echo "cloning frankapy"
RUN cd ~/git \ 
    && git clone --recurse-submodules https://github.com/Ruthrash/frankapy 

RUN echo "building frankapy python modules"
RUN cd ~/git/frankapy \
    && pip3 install empy==3.3.4 \
    &&  pip3 install -e .

RUN echo "compiling frankapy catkin_ws"
RUN cd ~/git/frankapy \ 
    && source /opt/ros/noetic/setup.bash \
    && ./bash_scripts/make_catkin.sh

RUN echo "compiling custom protobuf messages"
RUN cd ~/git/frankapy \ 
    && ./bash_scripts/make_proto.sh

RUN apt-get install -y \
    ros-noetic-libfranka \
    ros-noetic-franka-ros \
    ros-noetic-franka-gripper \
    ros-noetic-rosbash ros-noetic-ros-comm

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/git/frankapy/catkin_ws/devel/setup.bash " >> ~/.bashrc
 
RUN echo "export ROS_IP=${workstation_ip}" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://${workstation_ip}:11311" >> ~/.bashrc
RUN echo "export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python" >> ~/.bashrc
    

RUN pip3 install --upgrade protobuf

RUN pip3 install numpy --force-reinstall
#reinstalling numba seems to solve an error when we import FrankaArm from frankapy
RUN pip3 install zmq pyrealsense2 numba scipy open3d  opencv-python  wrapt_timeout_decorator
RUN if [[ "x$use_robotiq" == "x1" ]] ;\
    then echo "use_robotiq is set to TRUE" \
    && apt-get install ros-noetic-soem \
    && pip3 install -U pymodbus==2.5.3 empy==3.3.4 \
    && cd ~/git \
    && mkdir -p robotiq_ws/src \
    && cd robotiq_ws/src \
    && git clone https://github.com/pairlab/robotiq.git \
    && cd ../ \ 
    && source /opt/ros/noetic/setup.bash\
    && catkin build \
    && echo "source ~/git/robotiq_ws/devel/setup.bash --extend"  >> ~/.bashrc ; \
    fi

RUN if [[ "x$build_contactgraspnet" == "x1" ]] ;\
    then echo "build_contactgraspnet is set to TRUE" \
    && echo "Building conda for contact_graspnet" \
    && apt-get update && apt-get upgrade -y && apt-get install -y  freeglut3-dev\
    && cd /root/git \ 
    && git clone https://github.com/NVlabs/contact_graspnet \
    && cd contact_graspnet\
    && conda env create -f contact_graspnet_env.yml \
    && source activate contact_graspnet_env\
    && pip3 install open3d pyrealsense2\
    && cd ~/git/frankapy \
    # && pip install empy==3.3.4 \
    && pip3 install -e . \
    && conda deactivate\    
    && conda deactivate; \     
    fi

    
RUN if [[ "x$build_dope" == "x1" ]] ;\
    then echo "build_dope is set to TRUE" \
    && apt-get install ros-noetic-vision-msgs \ 
    && echo "Building conda for DOPE" \
    && cd /root/git && git clone https://github.com/NVlabs/Deep_Object_Pose.git dope \
    && export CONDA_DIR=/opt/conda && export PATH=$CONDA_DIR/bin:$PATH \
    && conda create --name dope python=3.8 \
    && source activate dope\ 
    && pip3 install catkin_pkg\  
    && cd dope && python3 -m pip install -r requirements.txt\
    && pip3 install -e . \ 
    && pip3 install torch torchvision --upgrade \ 
    && pip3 install numpy==1.20.3 \
    && pip3 install open3d pyrealsense2  \ 
    && cd ~/git/frankapy \
    # && pip install empy==3.3.4 \
    && pip3 install -e . \    
    && conda deactivate \
    && conda deactivate;\
    fi

RUN if [[ "x$build_megapose" == "x1" ]] ;\
    then echo "build_megapose is set to TRUE" \
    && cd /root/git && git clone --recurse-submodules https://github.com/megapose6d/megapose6d megapose \
    && export CONDA_DIR=/opt/conda && export PATH=$CONDA_DIR/bin:$PATH \
    && cd /root/git/megapose/conda \
    && sed -i 's/notebook/notebook==6.4.12/g' environment_full.yaml \ 
    &&  conda env create -f environment_full.yaml \
    && source activate megapose\    
    && cd /root/git/megapose && pip3 install -e . \ 
    && python -m megapose.scripts.download --megapose_models \
    && pip3 install torch torchvision --upgrade \ 
    # && pip3 install numpy==1.20.3 \
    && pip3 install open3d pyrealsense2 \ 
    && cd ~/git/frankapy \
    && pip install empy==3.3.4 \
    && pip3 install -e . \    
    && conda deactivate\
    && conda deactivate \ 
    && pip3 install rospkg  wrapt-timeout-decorator;\
    fi    



SHELL ["conda", "run", "-n", "myenv", "/bin/bash", "-c"]


