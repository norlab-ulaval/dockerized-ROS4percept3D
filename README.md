# ROS-4-Percept3D course software install 
Maintainer: luc.coupal.1@ulaval.ca

## ROS-4-Percept3D for virtual machine
Require os Ubuntu 20.04 (Focal)
Install ROS version Noetic

#### Script usage:
1. In the VM, execute the following line in a terminal
   ```shell
   cd /opt \
      && sudo apt-get update \
      && sudo apt-get install --assume-yes git \
      && sudo git clone https://github.com/norlab-ulaval/dockerized-ROS4percept3D.git \
      && cd dockerized-ROS4percept3D/ros-percept3D-4-VM \
      && sudo bash install_percept3d.bash
   ```
2. logout current user and login with user `student` pass `percept3d`

To connect remotely to the container:
```shell
# Find the VM_IP_ADDRESS using 
hostname -I | awk '{print $1}'

# Secure shell
ssh -p 2222 student@VM_IP_ADDRESS
# ftp access
sftp -P 2222 openssh-VM_IP_ADDRESS
# copy file
scp -P 2222 /path/to/foo student@VM_IP_ADDRESS:/dest/
```

[//]: # ssh -p 2222 student@10.211.55.11
[//]: # ssh -p 2222 student@10.37.129.1


(!) Be advise that VM root password as also been change to `percept3d`

**Note on unit test**: `docker pull --platform linux/arm64 ubuntu:20.04`

## ROS-4-Percept3D in Docker
Build either 
- Ubuntu 20.04 (Focal) with ROS version Noetic
- Ubuntu 18.04 (Bionic) with ROS version Melodic
(In progress)
