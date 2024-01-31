[![Docker](https://github.com/pairlab/franka_arm_infra/actions/workflows/main.yml/badge.svg)](https://github.com/pairlab/franka_arm_infra/actions/workflows/main.yml)
## Description 
This repository contains scripts and off-the-shelf starter scripts for developping and running algorithms for the Franka Emika Panda arms. 
- In the [docker](docker) folder you can find the docker compose yaml and Dockerfiles to setup the docker environment for the realtime computer and workstation computer interfacing with the Franka arms. [frankapy](https://github.com/iamlab-cmu/frankapy) and [frank-interface](https://github.com/iamlab-cmu/franka-interface) are also compiled in the docker environments and you should be able to use frankapy APIs out of the box through this repo. 

- In the [robot_toolkit](robot_toolkit) folder you can find the calibration scripts to run robot camera calibration routine(currently) and multi camera calibration routine (soon), find more info [here](robot_toolkit/docs). 

- In the [franka_control_suite](franka_control_suite)(Work in Progress) folder you can find experimental feedback controllers implemented in libfranka.

- In [tests](tests) (Work in Progress) folder we will provide various unit tests and integration test scripts to ensure the software system is working as expected. Including tests for the docker environment, frankapy, robot camera calibration, etc (Work in Progress)

## Dependancies 
- docker-compose (version 1 or version 2, use "docker-compose .." or "docker compose .." respectively)
- docker 
- git 
- openssh-server

```
sudo apt-get install docker.io docker-compose git 
```
install openssh-server, following instructions [here](https://www.cyberciti.biz/faq/ubuntu-linux-install-openssh-server/)

# Build Instructions

Before building the docker environments, you need to add your user to the docker group as mentioned [here](https://askubuntu.com/questions/477551/how-can-i-use-docker-without-sudo), so that you can run the docker commands without sudo preveileges and therefore need not type in your password everytime. 

```
sudo usermod -aG docker $USER
```
Note: If using docker-compose version 2, just replace commands containing "docker-compose" with "docker compose" 
installing docker dependancies 

**Note:** <u>**Realtime Computer**</u><a id='realtime'></a> is the computer that sends/receives data from/to the robot realtime(1Khz). It runs the realtime linux kernel as described [here](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). <u>**Workstation computer**</u><a id='workstation'></a> is the computer that sends high level commands to realtime computer to control the robot, this computer can run GPUs.

## [Real time Computer](#realtime)
Build docker container for the real time computer directly connected to Franka's control. 
```
docker-compose -f docker/realtime_computer/docker-compose-gui.yml build \
                            --build-arg workstation_ip=<workstation_ip address>\
                            --build-arg realtime_computer_ip=<realtime_computer_ip address>\
                            --build-arg robot_server_version=<robot_server_version>
docker-compose -f docker/realtime_computer/docker-compose-gui.yml create             
```
Note: To find your robot server version, first find your robot systems version in Franka Desk -> Settings -> System -> Version. Next find your robot server version corresponding to your system version [here](https://frankaemika.github.io/docs/compatibility.html#compatibility-with-libfranka). eg. for robot system version >=4.2.1, robot server version is 5. 

Note: While building the docker container, the above command might print warnings in red color, don't be alarmed and let the process run. If it stops building, that's when there is an error. 

### Build franka_control_suite in the realtime docker environment **(optional and not required to use frankapy)**
open a bash terminal inside the realtime docker container 
```
docker exec -it realtime_docker bash
```
go to franka_control_suite and build it
```
cd /root/git/franka_control_suite
mkdir build && cd build 
cmake ..
make
```

## [Workstation Computer](#workstation)
Build, create, and start the docker container for the workstation computer that has GPU/nvidia drivers 

**Note: it is important to pass the workstation IP address as seen by the Realtime computer here**
```
docker-compose -f docker/workstation_computer/docker-compose-gui.yml build \
                            --build-arg workstation_ip=<workstation_ip address>
docker-compose -f docker/workstation_computer/docker-compose-gui.yml create
docker-compose -f docker/workstation_computer/docker-compose-gui.yml start
                                                        
```
**Note:** The default CUDA version is 11.3 and Ubuntu version is 20.04, if you would like to change these, please aquire an appropriate docker image and use `--build-arg image=<your image>`.
**Note:** if you want to use [roboiq gripper](https://robotiq.com/products), please set `--build-arg use_robotiq=1` in the previous command for building workstation docker.

Note: While building the docker container, the above command might print warnings in red color, don't be alarmed and let the process run. If it stops building, that's when there is an error. 

## Setting up ssh-key between the workstation and realtime computers (done only once)<a id='ssh-key'></a>

Make sure to setup your workstation/workstation docker's ssh key to ssh into the realtime computer/docker without a password(this is required for frankapy) following instructions [here](https://github.com/iamlab-cmu/frankapy#setting-up-ssh-key-to-control-pc), you can run the following, 

1. In a terminal in your **<u>workstation/workstation docker</u>**, 
```
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
[Press enter]
[Press enter]
[Press enter]
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_rsa
```
2. Upload your public ssh key to the **<u>realtime pc</u>**
     
    i. In a separate terminal on your **<u>workstation PC/docker</u>**, use your favorite text editor to open your id_rsa.pub file.
    ```
    vim ~/.ssh/id_rsa.pub
    ```

    ii. In a new terminal on your **<u>workstation PC/docker</u>**, ssh to the realtime PC.
    ```
    ssh [realtime -pc-username]@[realtime -pc-name]
    Input password to realtime -pc.
    ```

    iii. Inside terminal in your realtime computer **(not realtime computer docker)**, 
    ```
    vim ~/.ssh/authorized_keys
    ```
    
    iv. Copy the contents from your id_rsa.pub file to a new line on the authorized_keys file on the real time. Then save

    v. Open a new terminal in the workstation docker and try sshing to the realtime PC and it should no longer require a password.

# Usage Instructions 

Note: docker-compose provides several commands to use the docker containers that we built in previous steps. They are ["up, start and run"](https://docs.docker.com/compose/faq/#whats-the-difference-between-up-run-and-start). In our docker containers we have not yet defined explicit services, so we will use "run". "up" creates or recreates the container(if you made changes to dockerfile or .yml files), therefore you might lose changes you made in the container, like [adding ssh-key](#ssh-key). As shown below we use the "start" command to run the container we built in the previous step, make sure to "stop" the container when done
## [Real time Computer](#realtime)
In the realtime host computer terminal, bring the built docker container up 
```
docker-compose -f docker/realtime_computer/docker-compose-gui.yml start
```
and when you are done with the container, run 
```
docker-compose -f docker/realtime_computer/docker-compose-gui.yml stop
```
this would get the container running. Then in a new terminal in the realtime host machine, run, 

To open a bash terminal inside the docker container 
```
docker exec -it realtime_docker bash
```

## [Workstation Computer](#workstation) 
In a terminal in the workstation computer
```
docker-compose -f docker/workstation_computer/docker-compose-gui.yml start
```
and when you are done with the container, run 
```
docker-compose -f docker/workstation_computer/docker-compose-gui.yml stop
```
In a new terminal in the workstation host machine, to allow GUI usage, first run, 
```
xhost +local:docker 
```

then to open a bash terminal inside the workstation docker container that we started running above, run
```
docker exec -it workstation_computer_docker bash
```

## Using frankapy 
Frankapy can be used with the real time docker and optionally with workstation_computer docker(if you don't use a docker for workstation, build and use [this frankapy](https://github.com/Ruthrash/frankapy)) 

**First** In your realtime pc, start the realtime computer docker with,
```
docker-compose -f docker/realtime_computer/docker-compose-gui.yml start
```
and when you are done with the container, run 
```
docker-compose -f docker/realtime_computer/docker-compose-gui.yml stop
```

if you are using workstation docker, **in a new terminal** start it with, 
as mentioned [here](https://stackoverflow.com/questions/69872788/docker-could-not-connect-to-any-x-display#:~:text=The%20solution%20is%20to%20run%20the%20following%20command%20in%20your%20terminal%3A) to get GUI access first run, 


```
xhost +local:docker 
```
then in the same terminal, run,
```
docker-compose -f docker/workstation_computer/docker-compose-gui.yml start
```
and when you are done with the container, run 
```
docker-compose -f docker/workstation_computer/docker-compose-gui.yml stop
```


To test the installation and setup of Frankapy,  

If using workstation docker, 

```
docker exec -it workstation_computer_docker bash
cd /root/git/frankapy 
bash ./bash_scripts/start_control_pc.sh -i (realtime computer ip) -u (realtimecomputer username) -d /root/git/franka-interface -a (robot_ip) -w (workstation IP)
```
to test, run
```
cd /root/git/tests/frankapy_control_test_scripts
python3 docker_frankapy_test.py
```
If directly using host workstation and not docker, 
```
cd <path to frankapy>/frankapy 
source catkin_ws/devel/setup.bash 
bash ./bash_scripts/start_control_pc.sh -i (realtime computer ip) -u (realtimecomputer username) -d /root/git/franka-interface -a (robot_ip) -w (workstation IP)
```
to test,
go to franka_arm_infra/tests directory in your workstation machine
```
cd <path to franka_arm_infra>/tests/frankapy_control_test_scripts
```
then run
```
python3 docker_frankapy_test.py
```

## Using calibration

Please checkout [robot_toolkit/docs](robot_toolkit/docs) for documentations of robot camera extrinsic calibration, usage of this tool. 


## Test robotiq gripper
To run the robotiq device, first in a terminal do
```sh
docker exec -it workstation_computer_docker bash
source ~/git/catkin_ws/devel/setup.bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```
**Note** when connecting the robotiq gripper to the workstation pc, it may open different file descriptor. In our case it was: `/dev/ttyUSB0`. You may check the following command to see which usb port the robotiq is connected to:  `ls /dev/ttyUSB*`

In another terminal do
```sh
docker exec -it workstation_computer_docker bash
source ~/git/catkin_ws/devel/setup.bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
```
then you can try first reset the gripper by passing `r` and then activate the gripper by passing `a`.

## Acknowledgements
- We thank [Reinhard Grasmann](https://reinhardgrassmann.github.io/) for providing CAD files that were super useful to run robot camera calibration routine, provided in this repo at [calibration/models_4_3d_printing](calibration/models_4_3d_printing)
