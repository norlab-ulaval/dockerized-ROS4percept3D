ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE} AS base-image
MAINTAINER luc.coupal.1@ulaval.ca

WORKDIR "/"

COPY beginner_tutorials.zip "/"
COPY percep3d_mapping.zip "/"
#RUN unzip beginner_tutorials.zip
#RUN unzip percep3d_mapping.zip

COPY install_percept3d.bash "/"

RUN bash install_percept3d.bash

CMD ["bash"]
