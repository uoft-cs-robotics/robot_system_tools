## Description 

### Instructions to install nvidia-container-toolkit in Ubuntu host machines 

Following instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
```

```
sudo nvidia-ctk runtime configure --runtime=docker
```
```
sudo systemctl restart docker
```


### Instructions to install cuDNN inside the docker container
- First download the tar file of cuDNN from the Nvidia website [here](https://developer.nvidia.com/cudnn-downloads) or the archive [here](https://developer.nvidia.com/rdp/cudnn-archive) 
- Copy the .tar file to the [scratchpad](../../docker/scratchpad/) directory in the host machine 
- Run install cudnn library script present in the scratchpad **inside the docker container's terminal**
  - The script basically moves the tar file to /usr/local and unzips it like so 
  ```
  sudo mv cudnn* /usr/local 
  cd  /usr/local 
  tar -xzvf cudnn* 
  sudo ldconfig
  ```