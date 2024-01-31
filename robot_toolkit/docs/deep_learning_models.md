## DOPE usage 

- The dockerfile creates a conda environment with all dependancies required. Make sure to activate the environment with 
```
conda activate dope
```
before you run test scripts that uses DOPE. 

- The model expects the weights to be stored in /root/git/dope/weights directory inside the docker. The way to use it is, first download the weights from [here](https://drive.google.com/drive/folders/1DfoA3m_Bm0fW8tOWXGVxi4ETlLEAgmcg) to [docker/scratchpad](../../docker/scratchpad) directory and inside the docker run, 
```
cd /root/git/scratchpad 
cp <weights file> /root/git/dope/weights
```

- Make sure to edit your config files as needed, you can see example of config file at [robot_toolkit/config/dope_config_pose.yaml](../../robot_toolkit/config/dope_config_pose.yaml). This file configured to run test_dope.py test script\


## Contact Graspnet Usage 
- Download the weights from [here](https://drive.google.com/drive/folders/1tBHKf60K8DLM5arm-Chyf7jxkzOr5zGl)
- Copy the weights file to [docker/scratchpad](../../docker/scratchpad) 


## Megapose Usage 
- Megapose models are already downloaded for you with a python command in the workstation/Dockerfile. 
- You would need meshes of the objects whose pose we want to detect. For tests we use YCB objects soupcan, drill and cheezit
- These meshes should be stored in [robot_toolkit/robot_arm_algos/src/inference/objects](../../robot_toolkit/robot_arm_algos/src/inference/objects/)
