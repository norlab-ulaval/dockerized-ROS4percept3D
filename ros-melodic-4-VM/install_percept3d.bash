#!/bin/bash
# /// ROS-4-Percept3D course software install //////////////////////////////////////////////////////////////////////////
# Maintainer: luc.coupal.1@ulaval.ca

# Note | docker pull --platform linux/arm64 ubuntu:20.04


ROS_PKG='desktop_full'
ROS_DISTRO='melodic'
DS_ROS_ROOT="/opt/ros/${ROS_DISTRO}"

# skip GUI dialog by setting everything to default
export DEBIAN_FRONTEND=noninteractive
ROS_DEV_WORKSPACE="${HOME}/catkin_ws"
PERCEPT_LIBRARIES="${HOME}/opt/percep3d_libraries"


mkdir -p "${DS_ROS_ROOT}"
mkdir -p "${ROS_DEV_WORKSPACE}"
mkdir -p "${PERCEPT_LIBRARIES}"

cd "${ROS_DEV_WORKSPACE}"

# install development utilities
apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        cmake \
        build-essential \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        ca-certificates \
        git \
        libusb-dev \
        usbutils \
        vim \
        tree \
        apt-utils \
        zip gzip tar unzip \
    && rm -rf /var/lib/apt/lists/*



# .... Install percept3D libraries dependencies ........................................................................
# . . Install boost. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# https://www.boost.org/doc/libs/1_79_0/more/getting_started/unix-variants.html
apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*


# . . Install eigen . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# . . Install libnabo . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        python-numpy \
    && rm -rf /var/lib/apt/lists/*

cd "${PERCEPT_LIBRARIES}"
# https://github.com/ethz-asl/libnabo
git clone https://github.com/ethz-asl/libnabo.git \
    && cd libnabo \
    && git checkout 1.0.7 \
    && mkdir build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc) \
    && make test \
    && make install

# ToDo:on task end >> next bloc ↓↓
pwd && tree -L 3

# . . Install percept3D libraries. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
cd "${PERCEPT_LIBRARIES}"

#apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        libyaml-cpp-dev \
#    && rm -rf /var/lib/apt/lists/*

# https://github.com/ethz-asl/libpointmatcher/tree/master
git clone https://github.com/ethz-asl/libpointmatcher.git \
    && cd libpointmatcher \
    && git checkout 1.3.1 \
    && mkdir build && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release \
#            -DCMAKE_INSTALL_PREFIX=/usr/local/ \
            -D BUILD_TESTS=TRUE \
             .. \
    && make -j $(nproc) \
    && make install

cd "${PERCEPT_LIBRARIES}/libpointmatcher/build"
utest/utest --path "${PERCEPT_LIBRARIES}/libpointmatcher/examples/data/"


cd "${PERCEPT_LIBRARIES}"
git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc) \
    && make install


# ToDo:on task end >> next bloc ↓↓
pwd && tree -L 3

# === ROS ==============================================================================================================

# ... register the ROS package source ..................................................................................
# Setup sources.lst
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# ... Install ROS,  ....................................................................................................

# Credit for the next two RUN step: NVIDIA-AI-IOT/ros2_jetson
#    https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

mkdir -p "${ROS_DEV_WORKSPACE}/src/"
cd "${ROS_DEV_WORKSPACE}/src/"
apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
#        ros-${ROS_DISTRO}-`echo "${ROS_PKG}" | tr '_' '-'` \
        ros-${ROS_DISTRO}-$(echo "${ROS_PKG}" | tr '_' '-') \
        python-rosdep \
        python-rosinstall-generator \
        python-vcstool \
        python-wstool \
        python-rosinstall \
        build-essential \
    && rosdep init \
    && rosdep update \
    && rosdep fix-permissions \
    && rm -rf /var/lib/apt/lists/*


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
cd "${ROS_DEV_WORKSPACE}"
apt-get update \
    && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro=${ROS_DISTRO} -y

#    && cd ${DS_DEV_WORKSPACE} \
#    && apt-get update \
#    && rosdep install --from-path src --ignore-src --default-yes \
#    && rm -rf /var/lib/apt/lists/*


/bin/bash -c "source ${DS_ROS_ROOT}/setup.bash \
    && catkin_make \
    && source ${ROS_DEV_WORKSPACE}/devel/setup.bash" \
    && echo "source ${DS_ROS_ROOT}/setup.bash" >> ~/.bashrc \
    && echo "source ${ROS_DEV_WORKSPACE}/devel/setup.bash" >> ~/.bashrc
# Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment
# variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros_catkin_ws/src:/opt/ros/melodic/share



## . . Pull required repository. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
cd "${ROS_DEV_WORKSPACE}/src/"
git clone https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
git clone https://github.com/norlab-ulaval/libpointmatcher_ros.git
git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git

# . . Add archived files . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#cd "${HOME}"
#cd "/"
mv "/beginner_tutorials.zip" "${ROS_DEV_WORKSPACE}/src"
mv "/percep3d_mapping.zip" "${ROS_DEV_WORKSPACE}/src"

#COPY beginner_tutorials.zip "${ROS_DEV_WORKSPACE}/src"
#COPY percep3d_mapping.zip "${ROS_DEV_WORKSPACE}/src"
unzip beginner_tutorials.zip
unzip percep3d_mapping.zip


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
cd "${ROS_DEV_WORKSPACE}"

# (Priority) ToDo:validate >> next bloc
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PERCEPT_LIBRARIES}/norlab_icp_mapper

apt-get update \
    && /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash \
    && catkin_make \
    && source ${ROS_DEV_WORKSPACE}/devel/setup.bash"


## Already install in ROS desktop full install
#apt-get update \
#    && apt-get install -y rviz


## .... install Paraview ................................................................................................
#FROM ros-base-image AS ROS4percept3DwParaView
#
## https://www.paraview.org
#apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#            python-dev \
#            libgl1-mesa-dev \
#            libxt-dev \
#            qt5-default \
#            libqt5x11extras5-dev \
#            libqt5help5 \
#            qttools5-dev \
#            qtxmlpatterns5-dev-tools \
#            libqt5svg5-dev \
#            libopenmpi-dev \
#            libtbb-dev \
#            ninja-build \
#            xvfb \
#            mesa-utils \
#            python-opengl \
#            ffmpeg \
##          python3-dev \
##          python3-numpy \
#    && rm -rf /var/lib/apt/lists/*
#
#
##wget https://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v5.8&type=binary&os=Linux&downloadFile=ParaView-5.8.1-MPI-Linux-Python2.7-64bit.tar.gz
#
## . . Install ParaView for Linux dependencies: LLVM. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
## Download the latest LLVM source: https://www.paraview.org/Wiki/ParaView_And_Mesa_3D
#curl -L -O https://github.com/llvm/llvm-project/releases/download/llvmorg-11.0.0/llvm-11.0.0.src.tar.xz \
#    && tar -xvf llvm-11.0.0.src.tar.xz \
#    && mkdir llvm-11.0.0.bld && cd llvm-11.0.0.bld \
#    && cmake -DCMAKE_BUILD_TYPE=Release                     \
#             -DCMAKE_INSTALL_PREFIX=$HOME/llvm/install      \
#             -DLLVM_BUILD_LLVM_DYLIB=ON                     \
#             -DLLVM_ENABLE_RTTI=ON                          \
#             -DLLVM_INSTALL_UTILS=ON                        \
#             -DLLVM_TARGETS_TO_BUILD="ARM;X86;AArch64"      \
#             ../llvm-11.0.0.src                             \
#    && make -j $(nproc) \
#    && make install
#
## . . Installing Mesa llvmpipe and swr drivers . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#
#
#cd "/opt"
#git clone https://gitlab.kitware.com/paraview/paraview.git \
#    && mkdir paraview_build  \
#    && cd paraview \
#    && git checkout v5.8.0 \
#    && git submodule update --init --recursive \
#    && cd ../paraview_build \
#    && cmake -GNinja -DPARAVIEW_USE_PYTHON=ON -DPARAVIEW_USE_MPI=ON -DVTK_SMP_IMPLEMENTATION_TYPE=TBB -DCMAKE_BUILD_TYPE=Release ../paraview \
#    && ninja


# Make sure that you have your environment properly setup. A good way to check is to ensure that environment variables
# like ROS_ROOT and ROS_PACKAGE_PATH are set:
#   $ printenv | grep ROS
cd "${ROS_DEV_WORKSPACE}"



# ////////////////////////////////////////////////////////////////////////// ROS-4-Percept3D course software install ///
