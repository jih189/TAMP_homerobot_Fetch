FROM ubuntu:18.04

WORKDIR /root/

# Add libcuda dummy dependency
ADD control .
RUN apt-get update && \
	apt-get install --yes equivs && \
	equivs-build control && \
	dpkg -i libcuda1-dummy_11.8_all.deb && \
	rm control libcuda1-dummy_11.8* && \
	apt-get remove --yes --purge --autoremove equivs && \
	rm -rf /var/lib/apt/lists/*

# Setup Lambda repository
ADD lambda.gpg .
RUN apt-get update && \
	apt-get install --yes gnupg && \
	apt-key add lambda.gpg && \
	rm lambda.gpg && \
	echo "deb http://archive.lambdalabs.com/ubuntu bionic main" > /etc/apt/sources.list.d/lambda.list && \
	echo "Package: *" > /etc/apt/preferences.d/lambda && \
	echo "Pin: origin archive.lambdalabs.com" >> /etc/apt/preferences.d/lambda && \
	echo "Pin-Priority: 1001" >> /etc/apt/preferences.d/lambda && \
	echo "cudnn cudnn/license_preseed select ACCEPT" | debconf-set-selections && \
	apt-get update && \
	DEBIAN_FRONTEND=noninteractive \
		apt-get install \
		--yes \
		--no-install-recommends \
		--option "Acquire::http::No-Cache=true" \
		--option "Acquire::http::Pipeline-Depth=0" \
		lambda-stack-cuda \
		lambda-server && \
	rm -rf /var/lib/apt/lists/*

# Setup for nvidia-docker
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0"

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

# # install jupyter
RUN python3 -m pip install --upgrade pip
RUN pip3 install --ignore-installed pyzmq && pip3 install jupyter

# install python2 kernel into jupyter notebook
RUN apt install python-pip -y
RUN python2 -m pip install ipykernel
RUN python2 -m ipykernel install

# Expose the port Jupyter will run on
EXPOSE 8888

# Install necessary for building
RUN apt-get update -q && \
    apt-get install -y ros-melodic-costmap-2d ros-melodic-rail-manipulation-msgs ros-melodic-joint-trajectory-controller ros-melodic-teleop-twist-keyboard

# Install fcl library
RUN git clone https://github.com/flexible-collision-library/fcl.git && cd fcl && mkdir build && cd build && cmake .. && make && make install

# install trimesh
RUN pip2 install trimesh
RUN apt-get install -y python-scipy python-rtree

# install blender
RUN apt-get update && apt-get install -y libc6 libc6-dev

COPY ./download/blender-3.4.0-linux-x64.tar.xz /root/

RUN tar -xf /root/blender-3.4.0-linux-x64.tar.xz -C /root && \
    rm /root/blender-3.4.0-linux-x64.tar.xz && cd /root && mv blender-3.4.0-linux-x64 Blender

# Download the Python Source since it is not bundled with Blender
RUN wget https://www.python.org/ftp/python/3.10.8/Python-3.10.8.tgz \
	&& tar -xzf Python-3.10.8.tgz \
	&& cp -r Python-3.10.8/Include/* /root/Blender/3.4/python/include/python3.10/ \
	&& rm -rf Python-3.10.8.tgz \
	&& rm -rf Python-3.10.8
	
# install ros related packages in blender
RUN /root/Blender/3.4/python/bin/python3.10 -m ensurepip && /root/Blender/3.4/python/bin/python3.10 -m pip install pyyaml rospkg opencv-python netifaces

# set alias for blender
RUN echo "alias blender='$HOME/Blender/blender'" >> $HOME/.bashrc

# install conda
# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p /opt/conda

# # Put conda in path so we can use conda activate
ENV PATH=$PATH:$CONDA_DIR/bin

# Create a conda env for contact graspnet
COPY ./download/contact_graspnet_env.tar.xz /root/
RUN tar -xf /root/contact_graspnet_env.tar.xz -C /root && rm /root/contact_graspnet_env.tar.xz
RUN conda env create -f contact_graspnet_env.yml

RUN echo 'source $CONDA_DIR/etc/profile.d/conda.sh' >> $HOME/.bashrc

# install python package for tensorflow
RUN apt-get install -y ros-melodic-ros-numpy 

# install python3 package required for ROS
RUN apt-get install -y python3-pip python3-yaml python3-netifaces && pip3 install rospkg catkin_pkg

# # recompile tf2_ros for python3
RUN apt-get install -y python3-catkin-pkg-modules python3-rospkg-modules python3-empy

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && mkdir -p $HOME/ros_python3_ws/src && cd $HOME/ros_python3_ws && wstool init && wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v melodic-devel && wstool up && rosdep install --from-paths src --ignore-src -y -r'
RUN sed -i '/from __future__ import absolute_import/aimport sys\nsys.path[:] = [x for x in sys.path if "python3" not in x] if sys.version_info[0] < 3 else sys.path' /root/ros_python3_ws/src/geometry2/tf2_ros/src/tf2_ros/__init__.py
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && cd $HOME/ros_python3_ws && \
	catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so'

# put the script to prepare the workspace
COPY prepare_workspace.sh $HOME

RUN echo 'source $HOME/catkin_ws/devel/setup.bash' >> $HOME/.bashrc
RUN echo 'source $HOME/ros_python3_ws/devel/setup.bash' >> $HOME/.bashrc

RUN mkdir -p $HOME/catkin_ws/src/jiaming_manipulation

WORKDIR /root