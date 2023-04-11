#!/bin/bash
cd catkin_ws
catkin_make
cp /root/catkin_ws/devel/lib/libsimExtRosControl.so $COPPELIASIM_ROOT_DIR
cp /root/catkin_ws/devel/lib/libsimExtRosServices.so $COPPELIASIM_ROOT_DIR
cd /root/catkin_ws/src/jiaming_manipulation/ros_tensorflow/src/contact_graspnet
conda run -n contact_graspnet_env sh compile_pointnet_tfops.sh