# Foliated Manifold Framework Tutorial

## Recall Foliation and Manifold
### Manifold
A manifold is a set of configurations under certain constraints. In this project, we will only consider the manifold that is a set of arm configuration statisfying some constraints.
### Foliation
A foliation is a set of manifolds defined by a common constraints with a list of co_parameters. 

## How to use the foliation framework
To use the foliation framework, you need to construct the foliated problem, so you can pass it to the task planner to generate the plan and use the motion planner to solve it.
### Construct foliation and intersection
In this project, we provide a template that user can over write them for their own use. The template is in foliated_base_class.py(<b>BaseFoliation</b> and <b>BaseIntersection</b>). To use it, you need to construct the foliation and intersection class. For example, if you want to construct a foliation and intersection for manipulation, you can do the following:

Each foliatin contains four variables:
1. <b>foliation name</b>: the name of the foliation.
2. <b>constraint parameters</b>: the constraint parameters of the foliation. Basically, this is just a directory storing the shared parameters of the constraints in foliation.
3. <b>co-parameters</b>: the list of co-parameter of the foliation. This is a list of co-parameters used to define a manifold with constraint function.
4. <b>similarity matrix</b>: the similarity matrix of the foliation. This is a matrix(_numpy array_) that defines the similarity between each two co-parameters. If two co-parameters are similar, then the corresponding manifolds are similar.

Therefore, to construct a foliation, you can define it as follows:
```
custom_foliation = BaseFoliation(
    foliation_name='custom_foliation', # string
    constraint_parameters=constraint_parameters, # dictionary
    co_parameters=co_parameters, # list
    similarity_matrix=similarity_matrix # numpy array
)
```
Based on user's definition, the save and load function must be over written as follows:
```
from foliated_base_class import BaseFoliation
# define the foliation class
class ManipulationFoliation(BaseFoliation):
    def save(self, dir_path):
        # save current foliation to file
    @staticmethod
    def load(self, dir_path):
        # load the foliation from file, and return the foliation
```
In manipulation, the intersection between manifolds may not be only a configuration. It can be anything such as a action or motion. For example, in pick and place task, the robot need to open gripper to transit from grasping manifold to ungrasping manifold. Therefore, we need to define the intersection class as follows:
```
from foliated_base_class import BaseIntersection
class ManipulationIntersection(BaseIntersection):
    def __init__(self, ...):
        # define the action of intersection
    def inverse(self):
        # define the inverse of intersection action
    def get_edge_configurations(self):
        # return the edge configurations of the intersection
    def save(self, file_path):
        # save the intersection to file
    @staticmethod
    def load(file_path):
        # load the intersection from file
```

### Define intersection sampling function
Generate a set of intersection manually is painful. Therefore, to define the intersection between foliations must be done by sampling. In this project, we provide a template. There are main three functions that need to be over written:
```
def prepare_sampling_function():
    # prepare the sampler.

def sampling_done_function():
    # delete the sampler.

def sampling_function(co_parameters_from_foliation1, co_parameters_from_foliation2):
    # sample the intersection.
    # both co_parameters_from_foliation1 and co_parameters_from_foliation2 are list of co_parameters from two foliations, then you need to provide how to sample here. As the result, the returned value must be a child class of BaseIntersection(the class defined by you previously, such as ManipulationIntersection).
``` 
Once you have defined all above functions, you can generate the intersection by calling the following function:
```
from foliated_base_class import FoliatedIntersection
foliated_intersection = FoliatedIntersection(
    foliation1=foliation1, # the first foliation
    foliation2=foliation2, # the second foliation
    sampling_function=sampling_function, # the sampling function
    prepare_sampling_function=prepare_sampling_function, # the function to prepare the sampling
    sampling_done_function=sampling_done_function # the function to delete the sampler
)
```

### Construct the foliated problem
Let's assume you have implemented the foliation and intersection class. Here we call them ManipulationFoliation and ManipulationIntersection. Then, you can construct the foliated problem as follows:

```
from foliated_base_class import FoliatedProblem
from manipulation_foliations_and_intersections import ManipulationFoliation, ManipulationIntersection 

foliated_problem = FoliatedProblem('problem name')
foliated_problem.set_foliation_n_foliated_intersection(list_of_foliations, list_of_foliated_intersections)
foliated_problem.sample_intersections(number_of_sampling_attempts)
```

### Save and load the foliated problem
To save the foliated problem, you can call the following function:
```
foliated_problem.save(dir_path)
```
To load the foliated problem, you can call the following function:
```
foliated_problem = FoliatedProblem.load(ManipulationFoliation, ManipulationIntersection, dir_path)
```
<b>Warning</b>

After loading the foliated problem, you can't call sampling again. You can only pass it to taskplanner to generate the plan.