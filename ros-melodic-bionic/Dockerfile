
# /// Dockerized-ROS4percept3D course //////////////////////////////////////////////////////////////////////////////////

#ARG BASE_IMAGE=osrf/ros:melodic-desktop-full-bionic
#ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1
ARG BASE_IMAGE=ubuntu:18.04
FROM ${BASE_IMAGE} AS ros-base-image
MAINTAINER luc.coupal.1@ulaval.ca

ARG ROS_PKG=desktop_full
ENV ROS_PKG=${ROS_PKG}
ENV ROS_DISTRO=melodic
ENV DS_ROS_ROOT=/opt/ros/${ROS_DISTRO}

# skip GUI dialog by setting everything to default
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DEV_WORKSPACE="${HOME}/catkin_ws"
ENV PERCEPT_LIBRARIES="${HOME}/opt/percep3d_libraries"
WORKDIR "${ROS_DEV_WORKSPACE}"

# install development utilities
RUN apt-get update \
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
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*


# . . Install eigen . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# . . Install libnabo . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . ..
# https://github.com/ethz-asl/libnabo
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        python-numpy \
#        cmake-gui \
    && rm -rf /var/lib/apt/lists/*

#WORKDIR "${PERCEPT_LIBRARIES}"
WORKDIR "/usr/local"
RUN git clone https://github.com/ethz-asl/libnabo.git \
    && cd libnabo \
    && git checkout 1.0.7 \
    #    && git checkout 3cab7eed92bd5d4aed997347b8c8a2692a83a532 \
    && mkdir build && cd build \
#    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo "${PERCEPT_LIBRARIES}/libnabo" \
    && cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo "/usr/local/libnabo" \
#    && cmake -D CMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc) \
    && make test \
    && make install

# ToDo:on task end >> next bloc ↓↓
RUN pwd && tree -L 3

# . . Install percept3D libraries. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# https://github.com/ethz-asl/libpointmatcher/tree/master
WORKDIR "${PERCEPT_LIBRARIES}"

#RUN apt-get update \
#    && apt-get install --assume-yes --no-install-recommends \
#        libyaml-cpp-dev \
#    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ethz-asl/libpointmatcher.git \
    && cd libpointmatcher \
#    && git checkout 76f99fce0fe69e6384102a0343fdf8d262626e1f \
    && git checkout 1.3.1 \
#    && git checkout devel/maplab \
    && mkdir build && cd build \
#    && cmake -D CMAKE_BUILD_TYPE=Release .. -D BUILD_TESTS=TRUE ${SRC_DIR} \
    && cmake -D CMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=/usr/local/ \
            -D BUILD_TESTS=TRUE \
             .. \
    && make -j $(nproc) \
    && make install

WORKDIR "${PERCEPT_LIBRARIES}/libpointmatcher/build"
RUN utest/utest --path "${PERCEPT_LIBRARIES}/libpointmatcher/examples/data/"


WORKDIR "${PERCEPT_LIBRARIES}"
RUN git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && mkdir -p norlab_icp_mapper/build && cd norlab_icp_mapper/build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc) \
    && make install


# ToDo:on task end >> next bloc ↓↓
RUN pwd && tree -L 3

# === ROS ==============================================================================================================

# ... register the ROS package source ..................................................................................
# Setup sources.lst
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# ... Install ROS,  ....................................................................................................

# Credit for the next two RUN step: NVIDIA-AI-IOT/ros2_jetson
#    https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros.noetic

WORKDIR "${ROS_DEV_WORKSPACE}/src/"
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        ros-${ROS_DISTRO}-`echo "${ROS_PKG}" | tr '_' '-'` \
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
WORKDIR "${ROS_DEV_WORKSPACE}"
RUN apt-get update \
    && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro=${ROS_DISTRO} -y

#    && cd ${DS_DEV_WORKSPACE} \
#    && apt-get update \
#    && rosdep install --from-path src --ignore-src --default-yes \
#    && rm -rf /var/lib/apt/lists/*


RUN /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash \
    && catkin_make \
    && source ${ROS_DEV_WORKSPACE}/devel/setup.bash" \
    && echo "source ${DS_ROS_ROOT}/setup.bash" >> ~/.bashrc \
    && echo "source ${ROS_DEV_WORKSPACE}/devel/setup.bash" >> ~/.bashrc
# Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment
# variable. It should include the directory you're in:
#   $ echo $ROS_PACKAGE_PATH
#   > /home/youruser/ros_catkin_ws/src:/opt/ros/melodic/share



## . . Pull required repository. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
WORKDIR "${ROS_DEV_WORKSPACE}/src/"
RUN git clone https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git
RUN git clone https://github.com/norlab-ulaval/libpointmatcher_ros.git
RUN git clone https://github.com/norlab-ulaval/percep3d_turtle_exercises.git

# . . Add archived files . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
COPY beginner_tutorials.zip "${ROS_DEV_WORKSPACE}/src"
COPY percep3d_mapping.zip "${ROS_DEV_WORKSPACE}/src"
RUN unzip beginner_tutorials.zip
RUN unzip percep3d_mapping.zip


# . . Install ROS & build catkin workspace. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
WORKDIR "${ROS_DEV_WORKSPACE}"

# (Priority) ToDo:validate >> next bloc
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PERCEPT_LIBRARIES}/norlab_icp_mapper

RUN apt-get update \
    && /bin/bash -c "source ${DS_ROS_ROOT}/setup.bash \
    && catkin_make \
    && source ${ROS_DEV_WORKSPACE}/devel/setup.bash"


## Already install in ROS desktop full install
#RUN apt-get update \
#    && apt-get install -y rviz


## .... install Paraview ................................................................................................
#FROM ros-base-image AS ROS4percept3DwParaView
#
## https://www.paraview.org
#RUN apt-get update \
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
##RUN wget https://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v5.8&type=binary&os=Linux&downloadFile=ParaView-5.8.1-MPI-Linux-Python2.7-64bit.tar.gz
#
## . . Install ParaView for Linux dependencies: LLVM. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
## Download the latest LLVM source: https://www.paraview.org/Wiki/ParaView_And_Mesa_3D
#RUN curl -L -O https://github.com/llvm/llvm-project/releases/download/llvmorg-11.0.0/llvm-11.0.0.src.tar.xz \
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
#WORKDIR "/opt"
#RUN git clone https://gitlab.kitware.com/paraview/paraview.git \
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
WORKDIR "${ROS_DEV_WORKSPACE}"
CMD [ "bash" ]


# ///////////////////////////////////////////////////////////////////////////////////////// norlab-mppi/dependencies ///



