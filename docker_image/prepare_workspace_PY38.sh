#!/bin/bash
cd catkin_ws
catkin_make
cp /root/catkin_ws/devel/lib/libsimExtRosControl.so $COPPELIASIM_ROOT_DIR
cp /root/catkin_ws/devel/lib/libsimExtRosServices.so $COPPELIASIM_ROOT_DIR
conda activate Tensorflow_env_PY38
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib/
cd /root/catkin_ws/src/jiaming_manipulation/ros_tensorflow/src/CoM_prediction
sh compile_pointnet_tfops_38.sh
cd /root/catkin_ws/src/jiaming_manipulation/ros_tensorflow/src/contact_graspnet
sh compile_pointnet_tfops_38.sh
conda deactivate