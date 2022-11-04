## Description 

## Dependancies 
- docker-compose
- docker 

installing docker dependancies 
```
sudo apt-get install docker.io docker-compose
```
## Build Instruction
# Real time Computer 
Build docker container for the real time computer directly connected to Franka's control 
```
sudo docker-compose --log-level ERROR  -f docker/realtime_computer/docker-compose-gui.yml build
```

# contrl Computer 
Build docker container for the control computer directly connected to Franka's control 
```
sudo docker-compose --log-level ERROR  -f docker/control_computer/docker-compose-gui.yml build
```


## Usage Instructions 
# Real time Computer 
Bring the built docker container up 

```
sudo docker-compose -f docker/realtime_computer/docker-compose-gui.yml up 
```

To open a bash terminal inside the docker container 
```
docker exec -it realtime_docker bash
```