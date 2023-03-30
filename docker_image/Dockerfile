FROM ubuntu:18.04

WORKDIR /root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -q && \
    apt-get install -y autotools-dev automake libtool python3-pip vim 

# include ros melodic
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

RUN echo 'export ROS_MASTER_URI=http://localhost:11311' >> $HOME/.bashrc
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
    rm /root/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04.tar.xz && cd /root && mv CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04 CoppeliaSim

ENV COPPELIASIM_ROOT_DIR=/root/CoppeliaSim

EXPOSE 19997

# install cmake
RUN apt-get update && \
    apt-get install -y build-essential wget && \
    wget https://github.com/Kitware/CMake/releases/download/v3.22.0/cmake-3.22.0.tar.gz && \
    tar -xf cmake-3.22.0.tar.gz && \
    rm cmake-3.22.0.tar.gz && \
    cd cmake-3.22.0 && \
    ./bootstrap && \
    make -j$(nproc) && \
    make install

# setup the python client for remote control in Coppeliasim
RUN python3 -m pip install pyzmq
RUN python3 -m pip install cbor
RUN git clone --recursive https://github.com/CoppeliaRobotics/zmqRemoteApi && cd zmqRemoteApi && git fetch && git checkout tags/coppeliasim-v4.4.0-rev0 && \
    mkdir -p build && cd build && cmake -DMAKE_BUILD_TYPE=Release .. && cmake --build . && cmake --install .

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

# install moveit
# setup the rosdep
RUN sudo apt -y install python-wstool python-catkin-tools clang-format-10 python-rosdep
RUN mkdir -p ws_moveit/src && cd ws_moveit/src && git clone https://github.com/jih189/moveit_cbirrt.git
RUN rosdep init && rosdep update
RUN cd ws_moveit && rosdep install -y --from-paths src --ignore-src --rosdistro melodic

# remove the ompl from ROS
RUN apt-get -y remove $(apt list --installed | awk -F/ '{print $1}' | grep ompl)

# reinstall the ompl in its build directory
RUN cd ompl/build/Release && make install

# compile moveit
RUN cd ws_moveit && catkin config --extend /opt/ros/melodic --install --install-space /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build

# install jupyter
RUN python3 -m pip install --upgrade pip
RUN pip3 install jupyter

# install python2 kernel into jupyter notebook
RUN apt install python-pip -y
RUN python2 -m pip install ipykernel
RUN python2 -m ipykernel install

# Expose the port Jupyter will run on
EXPOSE 8888

# Install necessary for building
RUN apt-get install -y ros-melodic-costmap-2d ros-melodic-rail-manipulation-msgs

# Install fcl library
RUN git clone https://github.com/flexible-collision-library/fcl.git && cd fcl && mkdir build && cd build && cmake .. && make && make install

# install trimesh
RUN pip2 install trimesh
RUN apt-get install -y python-scipy python-rtree

RUN mkdir -p $HOME/catkin_ws/src/jiaming_manipulation

WORKDIR /root