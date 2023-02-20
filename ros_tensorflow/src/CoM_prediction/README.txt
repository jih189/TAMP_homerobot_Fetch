src/CoM_predict/README.txt
Guide to running the CoM prediction server

1. conda env create -f CoM_Mass_Prediction_env.yml
2. export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib/
3. run the CoM_prediction_server node

Issues and solutions:
1. Require device=gpu: remember to run "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib/"
2. tensorflow.python.framework.errors_impl.NotFoundError: libtensorflow_framework.so.2: cannot open shared object file: No such file or directory or missing other tensorflow
    files: Try running bash compile_pointnet_tfops.sh
3. No option named std=c++11 or other compiler error when running compile_pointnet_tfops.sh: check that the cuda-toolkit version, driver version and nvcc version are compatible


Other possible issues:
1. grasp_prediction_server returns error related to GPU allocation: check line 32 in contact_graspnet/model.py, force it to use gpu:0
2. OOM error: try restarting CoM_prediction_server after starting grasp_prediction_server.If this still doesn't work, then there is no workaround :(
3. Always predicting CoMs very close to (0,0,0): check if the server is getting the pointcloud and parsing them correctly. Check node/rsonumpy_Com.py for more details