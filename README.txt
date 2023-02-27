To run experiments, run the run_experiments.py script in the base conda environment from the catkin_ws directory.
To change the logging file, go to line 368
To change the scenes being ran, go to the very end of the script and change the scenes being ran
run_for_scene(scene_name, constant_weights, variable_weights, num_trials)
  scene_name: the name of the scene being ran
  constant_weights: a bool value indicating whether to run the constant weight experiments
  variable_weights: a bool value indicating whether to run with different weights/mass
  num_trials: number of times to run the experiments with no-regrasp and regrasp
