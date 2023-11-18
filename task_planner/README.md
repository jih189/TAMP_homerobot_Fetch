# Task planner

Here is the task planner for manipulation in foliated structure problem. The paper is goint to be submitted to IRCA 2024 hopefully.

## Idea

The objective of this project is to leverage Gaussian Mixture Models (GMM) to model the self-collision-free space to enhance the performance of task and motion planning in foliated structure problem. In traditional task and motion planning, a two-level planner is commonly employed. Initially, the task planner devises a sequence of tasks, while the subsequent motion planner determines the motion plan for each task in the sequence. Prominent methods such as MTG (Informing Multi-Modal Planning with Synergistic Discrete Leads) and MDP (Multi-Modal Planning on Regrasping for Stable Manipulation) are widely used. However, these methods do not effectively reuse prior planning experiences in the motion planner. Therefore, our project aims to apply GMM to refine the task graph into a more detailed level. This way, we can convey previous information as a sequence of distributions to assist the motion planner. Additionally, we also plan to incorporate deep learning techniques to analyze the point cloud data and predict the feasibility of each pre-defined distribution.

## Table of Contents

- [Task planner](#task-planner)
  - [Idea](#idea)
  - [Goal](#goal)
  - [Foliated Problem Problem Construction](#foliated-problem-problem-construction)
  <!-- - [Experiments](#experiments) -->
  - [Nautilus](#nautilus)
  - [Usage](#usage)
    - [Base tutorial](#base-tutorial)
    - [Evaluation tutorial](#evaluation-tutorial)
    - [Dataset generation tutorial](#dataset-generation-tutorial)
    - [To download the dataset(internal use only)](#to-download-the-datasetinternal-use-only)
    - [Motion planning tutorial](#motion-planning-tutorial)

## Goal
- compare MTG and MDP
- compare MTG and MDP with GMM
- compare MTG and MDP with deep learning based GMM (Optional)
- compared MTG and MDP with GMM and constraint preprocess (Optional)

## Foliated Problem Problem Construction

The tutorial is in here [foliated manifold framework setup](foliation_setup_tutorial.md)

<!-- ## Experiments

Here is the list of experiment we will have for this project:

1. pick-and-place with constraint
2. pick-and-place with constraint and regrasping
3. Sliding and regrasping(IROS 2023)
4. Sliding in simple maze

Each experiment here will be saved as a file in directory [experiment_dir](experiment_dir) so later we can load them for testing. Thus, you may need to read the comment in the file to understand how to save and load them. To create the experiment file, you can use the following code:
```
rosrun task_planner create_experiment_[experiment-name].py
``` -->

### Future work
In the future, we will implement a code to read a yaml file describing the experiment and generate the experiment file. This way, we can easily create a new experiment.

## Nautilus
To get access to PRP Nautilus cluster. You need to install [kubectl tool](https://docs.nationalresearchplatform.org/userdocs/start/quickstart/). Then, for this project, jiaming has created an account for using that, so you can ask him for the config for this project. Once you got it, create a .kube directory and place the config find into it.
```
mkdir ~/.kube
cd .kube
# place the 'config' file here.
```

Run the following code to check whether you can access
```
kubectl get pods | grep jiaming
```
You should see a pod named 'jiaming-http-....'. If you can't find it, then you can ask jiaming to launch the container.

To enter it, you can use the following code to enter the container. You should replace the ... with what you see in the above command.
```
kubectl exec -it jiaming-http-... -- /bin/bash
```

Once you enter the container, you should build the jiaming-manipulation workspace by the following code
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/jih189/jiaming_manipulation.git
cd jiaming_manipulation
rm -r manipulation_test rail_segmentation
cd ../..
catkin build
source devel/setup.bash
```
There are some packages you should delete because we need simplify the docker container.

## Usage

### Base tutorial

In this project, we do not need to run on the real robot, so we can run the move_group in only simulation. Here is the code to launch fake move_group.

```
roslaunch fetch_moveit_config fake_move_group.launch
```

Then you can run the following code for the main pipeline
```
rosrun task_planner main_pipeline.py
```

### Evaluation tutorial

We also provide the code to evaluate different task planner. 
```
rosrun task_planner evaluation.py
```

This code will first load the experiment file in experiment_dir, then evaluate the performance of each task planner. The evaluation result will be saved in the directory jiaming_manipulation/taskplanner/evaluated_data_dir. You should modify the parameter in the file to change the experiment you want to evaluate and the task planner you want to use. After evaluation, you can visualize them with the following code
```
rosrun task_planner visualize_result.py
```

### Dataset generation tutorial

In this project, we have a predictor to read the pointcloud and predict the feasibility of each predefined distribution of GMM. For this purpose, we need to generate the dataset. Here, we assume we have GMM already. (you need to have a fake move group launching for the following command)

First, generate a set of self-collision-free arm configurations with following code.
```
rosrun data_generation random_joint_state_generation.py
```
This code will generate a directory(jiaming_manipulation/data_generation/gmm_data) with joint_names.npy and valid_robot_states.npy. joint_names.py contains the name of all joint of the robot, while valid_robot_state.npy contains all valid robot state(only joint values).

Second, based on those self-collision-free arm configurations, we randomly generate a scene and valid tag with the following command:
```
rosrun data_generation pointcloud_joint_state_validity_generation.py _random_value:=<random_value> _total_env_num:=<number of scene to generate>
```
Here, the random value is used as seed, while the total_env_num is the number of scene you want to generate. After generation, for each scene i, there will be a dir named "env_i". In this directory, there are two files. map_i.ply is the pointcloud of the obstacle, while valid_tag.npy is a vector of valid flag for each arm configuration in the valid_robot_states.npy.

Third, load the gmm and predict the feasibility of each predefined distribution with the following command:
```
rosrun task_planner prepare_gmm_dataset.py
```

This command will first load the gmm, then find the gmm_data in the data_generation, the read the valid_robot_states.npy. Based on the gmm and valid_tag.npy in each env_i, it can calculate the total number of sampled joint state of each distribution as total_count_dic.npy. Then, it also produces the valid number of each distribution as valid_count_dic.npy. Both of them are dictionary.

### To download the dataset(internal use only)
We use s3cmd to save all data, so you need to download the s3cmd first.
```
apt-get install s3cmd
```
Then you can put the configuration file into ~/.s3cfg. Then, you can list all files in the s3 bucket with the following command:
```
s3cmd ls s3://my-bucket
```
It should print out all files in the bucket like this:
```
2023-08-07 06:37  66233424   s3://my-bucket/gmm-data-0000
2023-08-07 22:28  66188139   s3://my-bucket/gmm-data-1000
2023-08-07 06:39  66214753   s3://my-bucket/gmm-data-2000
2023-08-07 06:46  66256142   s3://my-bucket/gmm-data-3000
2023-08-07 06:27  66182459   s3://my-bucket/gmm-data-4000
2023-08-07 06:30  66210384   s3://my-bucket/gmm-data-5000
2023-08-07 06:32  66216336   s3://my-bucket/gmm-data-6000
2023-08-07 06:33  66204653   s3://my-bucket/gmm-data-7000
2023-08-05 17:38    990042   s3://my-bucket/my-folder
```
Then you can download each of them with the following command (for example, you want to download gmm-data-0000):
```
s3cmd get s3://my-bucket/gmm-data-0000
```
or you can use the following command to download all of them (In this project, you should download all of them):
```
s3cmd get s3://my-bucket --recursive
```
You can unzip and merge them together with the following command into gmm_data:
```
unzip my-folder && 
unzip gmm-data-0000 &&
unzip gmm-data-1000 &&
unzip gmm-data-2000 &&
unzip gmm-data-3000 &&
unzip gmm-data-4000 &&
unzip gmm-data-5000 &&
unzip gmm-data-6000 &&
unzip gmm-data-7000 
```

### Motion planning tutorial
In this project, we will use CDistributionRRT which will read both constraints and distribution to plan the motion trajectory. We also provide an example code CDistributionRRT_example.ipynb in jupyter_note_tests.

For more details, 

1. To use CDistributionRRT, you must set path constraint, or it will cause error.
2. You can save SamplingDistribution into a list and pass it to motion planner with function set_distribution
3. To set sample ratio p to define the probability to sample uniformly(1-p) or with distribution(with p), you can modify the value of sample_ratio under "CDISTRIBUTIONRRTConfigDefault" in fetch_ros/fetch_moveit_config/config/ompl_planning.yaml.