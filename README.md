# ROS-4-Percept3D course software install 
Maintainer: luc.coupal.1@ulaval.ca

## ROS-4-Percept3D for virtual machine
#### Requirement
- os Ubuntu 20.04 (Focal)

#### Script will install: 
- ROS version: Noetic
- `libpointmatcher` (latest) + dependencies (boost, eigen, ANN, FLANN, libnabo)
- Required repository for the _Perception3D_ course: 
  - norlab_icp_mapper
  - norlab_icp_mapper_ros
  - libpointmatcher_ros
  - percep3d_turtle_exercises
- A builded catkin workspace 
- The course required directory structure
- The user `student` with proper privilege
- A configured ssh server for remote VM access

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
3. ★ Be advise that VM root password as also been change to `percept3d`

#### To connect remotely to the container:
1. first in the VM, open a terminal and execute 
   ```shell
   # Find the VM_IP_ADDRESS using 
   hostname -I | awk '{print $1}'
   ```
2. In the host computer, open a terminal and execute
   ```shell
   # Secure shell
   ssh -p 2222 student@VM_IP_ADDRESS
   
   # or to copy file
   scp -P 2222 /path/to/foo student@VM_IP_ADDRESS:/dest/
   ```


**Note on unit test on aarch arm64**: `docker pull --platform linux/arm64 ubuntu:20.04`

## ROS-4-Percept3D in Docker (In progress)
Build either 
- Ubuntu 20.04 (Focal) with ROS version Noetic
- Ubuntu 18.04 (Bionic) with ROS version Melodic


