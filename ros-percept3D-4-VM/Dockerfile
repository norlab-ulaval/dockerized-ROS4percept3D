ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE} AS base-image
MAINTAINER luc.coupal.1@ulaval.ca

WORKDIR "/"

ARG DEBIAN_FRONTEND=noninteractive

# install development utilities
RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        apt-utils \
        sudo \
        gnupg2 \
        lsb-release \
        cmake \
        build-essential \
        curl \
        wget \
        ca-certificates \
        git \
        libusb-dev \
        usbutils \
        vim \
        tree \
        zip gzip tar unzip \
    && rm -rf /var/lib/apt/lists/*

#RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo
#USER docker

COPY beginner_tutorials.zip "."
COPY percep3d_mapping.zip "."
#RUN unzip beginner_tutorials.zip
#RUN unzip percep3d_mapping.zip

COPY install_percept3d.bash "/"


# set read/write permission to entrypoint file
RUN /bin/bash -c "chmod +x /install_percept3d.bash"
RUN /bin/bash -c "/install_percept3d.bash"
#ENTRYPOINT [ "/install_percept3d.bash" ]
CMD ["bash"]
