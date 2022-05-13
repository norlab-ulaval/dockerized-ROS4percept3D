# ROS-4-Percept3D course software install 
Maintainer: luc.coupal.1@ulaval.ca

## ROS-4-Percept3D for virtual machine
#### Script usage:
1. In the VM, execute the following line in a terminal
   ```shell
   $ cd /opt
   $ sudo apt-get update && sudo apt-get install --assume-yes git
   $ sudo git clone https://github.com/norlab-ulaval/dockerized-ROS4percept3D.git
   $ cd dockerized-ROS4percept3D/ros-percept3D-4-VM
   $ sudo bash install_percept3d.bash
   ```
2. logout current user and login with user `student` pass `percept3d`

(!) Be advise that VM root password as also been change to `percept3d`

**Note on unit test**: `docker pull --platform linux/arm64 ubuntu:20.04`

## ROS-4-Percept3D in Docker
(In progress)
