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

Troubleshooting: 
- If the inference time for contactgraspnet is very high(in minutes) or you notice that the predicted grasps are way off, this means that your tensorflow version could be off as reported [here](https://github.com/NVlabs/contact_graspnet/issues/9)
- To solve this, 
    - First, consider getting the right cuda version for your GPU, if you think the version is not matching with what is installed in the docker (see line 1 in [Dockerfile](../../docker/workstation_computer/Dockerfile))
    - You can edit above mentioned line 1 with the image name that has the right CUDA version [here](https://hub.docker.com/r/nvidia/cuda/tags)
    - Upgrade tensorlow by running (make sure the conda environment for contact graspnet is active) ```pip3 install tensorflow --upgrade```
    - The opensourced contact graspnet model is compiled with cuDNN 8.1.1 so we need to install that as well, following instructions in [here](./nvidia-container-toolkit.md)
    - recompile pointnet model as mentioned [here](https://github.com/NVlabs/contact_graspnet?tab=readme-ov-file#troubleshooting) 
    - This shold solve the above menntioned problem
    - Note: We tested contactgraspnet with tensorflow 2.11, CUDA 11.1.1, CUDNN 8.1.1

## Megapose Usage 
- Megapose models are already downloaded for you with a python command in the workstation/Dockerfile. 
- You would need meshes of the objects whose pose we want to detect. For tests we use YCB objects soupcan, drill and cheezit
- These meshes should be stored in [robot_toolkit/robot_arm_algos/src/inference/objects](../../robot_toolkit/robot_arm_algos/src/inference/objects/)
