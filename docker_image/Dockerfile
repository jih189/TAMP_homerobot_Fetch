FROM ubuntu:18.04

# include ros melodic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
   apt-get install -y --no-install-recommends \
         git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
      net-tools \
  && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && \
   apt-get install -y --no-install-recommends \
		ros-melodic-desktop-full \
		ros-melodic-image-transport \
		ros-melodic-vision-msgs \
		ros-melodic-message-filters \
         python-rosdep \
         python-rosinstall \
         python-rosinstall-generator \
         python-wstool \
   && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/melodic/setup.bash' >> $HOME/.bashrc

RUN echo 'export ROS_MASTER_URI=https://localhost:11311' >> $HOME/.bashrc
RUN echo 'export ROS_HOSTNAME=localhost' >> $HOME/.bashrc

# # install the coppeliasim

RUN apt-get update -q && \
	export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
        vim tar xz-utils \
        libx11-6 libxcb1 libxau6 libgl1-mesa-dev \
        xvfb dbus-x11 x11-utils libxkbcommon-x11-0 \
        libavcodec-dev libavformat-dev libswscale-dev \
	libsodium23 libopenexr22 \
	libraw1394-11 libraw1394-dev libraw1394-tools \
	libusb-1.0-0 libusb-1.0-0-dev \
        && \
    apt-get autoclean -y && apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /shared /opt

COPY ./download/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04.tar.xz /root/
RUN tar -xf /root/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04.tar.xz -C /root && \
    rm /root/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04.tar.xz

ENV COPPELIASIM_ROOT_DIR=/root/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04
# ENV LD_LIBRARY_PATH=$COPPELIASIM_ROOT_DIR:$LD_LIBRARY_PATH
# ENV PATH=$COPPELIASIM_ROOT_DIR:$PATH

EXPOSE 19997

# # RUN echo '#!/bin/bash\ncd $COPPELIASIM_ROOT_DIR; coppeliaSim "$@"' > /entrypoint && chmod a+x /entrypoint
# # ENTRYPOINT ["/entrypoint"]

RUN apt-get update -q && \
    apt-get install -y autotools-dev automake libtool python3-pip vim 

WORKDIR /root

# install ompl

# first, need to instal ODE
RUN git clone https://github.com/thomasmarsh/ODE.git
RUN cd ODE && mkdir -p build && cd build && cmake .. && make && make install

# second, need to install Py++
COPY ./download/castxml.tar.xz $HOME
RUN tar -xf $HOME/castxml.tar.xz -C $HOME && rm $HOME/castxml.tar.xz
ENV PATH=/root/castxml/bin:${PATH}

RUN sudo -H pip3 install -vU pygccxml pyplusplus numpy

# then install ompl
RUN git clone https://github.com/ompl/ompl.git && cd ompl && mkdir -p build/Release && cd build/Release && \
    cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic ../.. -DPYTHON_EXEC=/usr/bin/python3  -DCASTXML=/root/castxml/bin/castxml && make update_bindings && make && make install