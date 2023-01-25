## Description 


## Dependancies 
- docker-compose
- docker 
- git 

installing docker dependancies 
```
sudo apt-get install docker.io docker-compose
```
## Build Instruction


Note: Realtime Computer is the computer that sends/receives data from/to the robot realtime(1Khz). this runs the realtime linux kernel. Workstation computer is the computer that sends high level control commands to the robot, this computer can run GPUs

# Real time Computer 
Build docker container for the real time computer directly connected to Franka's control 
```
sudo docker-compose --log-level ERROR  -f docker/realtime_computer/docker-compose-gui.yml build
```

# Workstation Computer 
Build docker container for the workstation computer that has GPU/nvidia drivers
```
sudo docker-compose --log-level ERROR  -f docker/workstation_computer/docker-compose-gui.yml build
```


## Usage Instructions 
# Workstation Computer 
Bring the built docker container up 

```
sudo docker-compose -f docker/workstation_computer/docker-compose-gui.yml up 
```

To open a bash terminal inside the docker container 
```
docker exec -it workstation_computer_docker bash
```

# To use frankapy 
Frankapy can be used with the real time computer docker and optionally with workstation_computer docker(if you don't use a docker for workstation, build and use [this frankapy](https://github.com/Ruthrash/frankapy)) 

In your realtime pc, start the realtime computer docker with,
```
sudo docker-compose -f docker/realtime_computer/docker-compose-gui.yml up 
```

Optionally, if you are using workstation docker, start it with, 
```
sudo docker-compose -f docker/workstation_computer/docker-compose-gui.yml up 
```

Make sure to setup your workstation/workstation docker's ssh key to ssh without a password(this is needed for frankapy) following instructions [here](https://github.com/iamlab-cmu/frankapy#setting-up-ssh-key-to-control-pc)

start frankapy by 

If using workstation docker, 

```
docker exec -it workstation_computer_docker bash
cd /root/git/frankapy 
bash ./bash_scripts/start_control_pc.sh -i (realtime computer ip) -u (realtimecomputer username) -d /root/git/franka-interface -a (robot_ip) -w (workstation IP)
```
to test, run
```
python3 scripts/reset_arm.py
```
If directly using host workstation and not docker, 
```
cd (frankapy path)/frankapy 
source catkin_ws/devel/setup.bash 
bash ./bash_scripts/start_control_pc.sh -i (realtime computer ip) -u (realtimecomputer username) -d /root/git/franka-interface -a (robot_ip) -w (workstation IP)
```
to test, run in the workstation computer/docker
```
python3 scripts/reset_arm.py
```

# Troubleshooting
- permission denied when opening a bash terminal insider the docker environment like so- 
```
Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Get "http://%2Fvar%2Frun%2Fdocker.sock/v1.24/containers/workstation_computer_docker/json": dial unix /var/run/docker.sock: connect: permission denied
```
Follow instructions [here](https://stackoverflow.com/questions/48957195/how-to-fix-docker-got-permission-denied-issue#:~:text=If%20you%20want%20to%20run%20docker%20as%20non%2Droot%20user%20then%20you%20need%20to%20add%20it%20to%20the%20docker%20group.) 
If you want to run docker as non-root user then you need to add it to the docker group.