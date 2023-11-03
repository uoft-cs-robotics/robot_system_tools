ARG image=nvidia/cudagl:11.3.0-devel-ubuntu20.04
FROM ${image}

ARG DEBIAN_FRONTEND=noninteractive
ARG workstation_ip=192.168.0.1
ARG use_robotiq=0
ARG build_dope=1
ARG build_contactgraspnet=1
ARG build_megapose=1
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
    ros-noetic-franka-gripper \
    ros-noetic-rosbash ros-noetic-ros-comm

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


RUN if [[ "x$build_contactgraspnet" == "x1" ]] ;\
    then echo "build_contactgraspnet is set to TRUE" \
    && echo "Building conda for contact_graspnet" \
    && apt-get update && apt-get upgrade -y && apt-get install -y  freeglut3-dev\
    && export CONDA_DIR=/opt/conda \
    && wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda \ 
    && export PATH=$CONDA_DIR/bin:$PATH \ 
    && cd /root/git \ 
    && git clone https://github.com/NVlabs/contact_graspnet \
    && cd contact_graspnet\
    && conda env create -f contact_graspnet_env.yml \
    && conda init bash ; \
    fi

    
RUN if [[ "x$build_dope" == "x1" ]] ;\
    then echo "build_dope is set to TRUE" \
    && echo "Building conda for DOPE" \
    && cd /root/git && git clone https://github.com/NVlabs/Deep_Object_Pose.git dope \
    && export CONDA_DIR=/opt/conda && export PATH=$CONDA_DIR/bin:$PATH \
    && conda create --name dope python=3.8 \
    && source activate dope\    
    && conda install -c conda-forge ros-rospy \
    && cd dope && python3 -m pip install -r requirements.txt\
    && pip3 install -e . \ 
    && pip3 install torch torchvision --upgrade \ 
    && pip3 install numpy==1.20.3 \
    && pip3 install open3d pyrealsense2 \ 
    && conda deactivate;\
    fi

# RUN if [[ "x$build_megapose" == "x1" ]] ;\
#     then echo "build_dope is set to TRUE" \
#     && cd /root/git && git clone https://github.com/megapose6d/megapose6d megapose \
#     && export CONDA_DIR=/opt/conda && export PATH=$CONDA_DIR/bin:$PATH \
#     && cd /root/git/megapose/conda \
#     && sed -i 's/notebook/notebook==6.4.12/g' environment_full.yaml \ 
#     &&  conda env create -f environment_full.yaml \
#     && source activate megapose\    
#     && pip3 install -e . \ 
#     && apt-get install chromium-browser\
#     && apt-get -y install chromium-chromedriver\
#     && python -m megapose.scripts.download --megapose_models \
#     && conda install -c conda-forge firefox geckodriver
#     && pip3 install torch torchvision --upgrade \ 
#     # && pip3 install numpy==1.20.3 \
#     && pip3 install open3d pyrealsense2 \ 
#     && conda deactivate;\
# && pip3 install rospkg  wrapt-timeout-decorator

#     fi    

# RUN apt-get install -y wget
# RUN wget -q https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
# RUN apt-get install -y ./google-chrome-stable_current_amd64.deb

# pip install webdriver-manager

# # Adding Google Chrome to the repositories
# RUN sh -c 'echo "deb [arch=x86_64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'

# # Updating apt to see and install Google Chrome
# RUN apt-get -y update

# # Magic happens
# RUN apt-get install -y google-chrome-stable

# # Installing Unzip
# RUN apt-get install -yqq unzip

# # Download the Chrome Driver
# RUN wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add -
# RUN sh -c 'echo "deb [arch=x86_64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'
# RUN apt-get -y update
# RUN apt-get install -y google-chrome-stable

# # install chromedriver
# RUN apt-get install -yqq unzip
# RUN wget -O /tmp/chromedriver.zip http://chromedriver.storage.googleapis.com/`curl -sS chromedriver.storage.googleapis.com/LATEST_RELEASE`/chromedriver_linux64.zip
# RUN unzip /tmp/chromedriver.zip chromedriver -d /usr/local/bin/

# # Set display port as an environment variable
# ENV DISPLAY=:99

# COPY ./app ./app
# WORKDIR /app 

# RUN pip install --upgrade pip

# RUN pip install -r requirements.txt

#######################################################################
# RUN apt-get install -y software-properties-common 
# RUN add-apt-repository -y ppa:openjdk-r/ppa
# RUN apt-get install -y openjdk-12-jre cron wget unzip

# ARG CHROME_VERSION=78.0.3904.87-1
# RUN wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add - \
#     && echo "deb http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list \
#     && apt-get update -qqy \
#     && apt-get -qqy install google-chrome-stable=$CHROME_VERSION \
#     && rm /etc/apt/sources.list.d/google-chrome.list \
#     && rm -rf /var/lib/apt/lists/* /var/cache/apt/* \
#         && sed -i 's/"$HERE\/chrome"/"$HERE\/chrome" --no-sandbox/g' /opt/google/chrome/google-chrome


# ARG CHROME_DRIVER_VERSION=78.0.3904.70
# RUN wget --no-verbose -O /tmp/chromedriver_linux64.zip https://chromedriver.storage.googleapis.com/$CHROME_DRIVER_VERSION/chromedriver_linux64.zip \
#     && rm -rf /opt/chromedriver \
#     && unzip /tmp/chromedriver_linux64.zip -d /opt \
#     && rm /tmp/chromedriver_linux64.zip \
#     && mv /opt/chromedriver /opt/chromedriver-$CHROME_DRIVER_VERSION \
#     && chmod 755 /opt/chromedriver-$CHROME_DRIVER_VERSION \
#     && ln -fs /opt/chromedriver-$CHROME_DRIVER_VERSION /usr/bin/chromedriver
#######################################################################

# RUN if [[ "x$build_dope" == "x1" ]] ;\
#     then echo "build_dope is set to TRUE" \
#     && echo "Building conda for DOPE" \
#     && cd /root/git && mkdir catkin_ws/src -p && cd catkin_ws/src \
#     && source /opt/ros/noetic/setup.bash \
#     && apt-get install -y ros-noetic-vision-msgs \
#     && catkin_init_workspace && git clone https://github.com/NVlabs/Deep_Object_Pose.git dope \
#     && export CONDA_DIR=/opt/conda && export PATH=$CONDA_DIR/bin:$PATH \
#     && conda create --name dope_env python=3.8 \
#     # && source activate dope_env\    
#     && conda install -c conda-forge ros-rospy \
# conda install -c conda-forge python-chromedriver-binary
# ln -s /opt/conda/envs/megapose/lib/python3.9/site-packages/chromedriver_binary/ /bin/chromedriver

# pip install chromedriver-autoinstaller
# pip install webdriver-manager

#     && cd dope && python3 -m pip install -r requirements.txt\
#     && cd /root/git/catkin_ws \
#     # && conda deactivate \    
#     # we ask catkin to use dope's conda environment python interpretor
#     && catkin_make -DPYTHON_EXECUTABLE=/opt/conda/envs/dope_env/bin/python3 ;\
#     fi


SHELL ["conda", "run", "-n", "myenv", "/bin/bash", "-c"]


# && rosdep install --from-paths src -i --rosdistro ${ROS_DISTRO} \