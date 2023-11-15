#!/usr/bin/env python
# from experiment_scripts.experiment_helper import Experiment, Manifold, Intersection

# user needs to implement this function
class BaseIntersection(object):
    """
        This class represents a base intersection.
        It is an abstract class. It is used to represent the intersection of two manifolds.
        User needs to implement both get and inverse functions.
    """
    def __init__(self, intersection_motion):
        """Constructor for BaseIntersection class, user does not need to implement this function"""
        self.intersection_motion = intersection_motion

    def get(self):
        """Return the intersection, user needs to implement this function"""
        # Return the intersection
        raise NotImplementedError("Please Implement this method")

    def inverse(self):
        """Return the inverse of the intersection, user needs to implement this function"""
        # Return inversed the intersection
        raise NotImplementedError("Please Implement this method")

class BaseFoliation:
    def __init__(self, foliation_name, constraint_parameters, co_parameters = []):
        if not isinstance(co_parameters, list):
            raise Exception("co_parameters is not a list")
        if co_parameters.__len__() == 0:
            raise Exception("co_parameters is empty")
        # check if constraint_parameters is a dictionary
        if not isinstance(constraint_parameters, dict):
            raise Exception("constraint_parameters is not a dictionary")
        # check if constraint_parameters is empty
        if constraint_parameters.__len__() == 0:
            raise Exception("constraint_parameters is empty")

        self.foliation_name = foliation_name
        self.constraint_parameters = constraint_parameters # constraint_parameters is a set of constraint parameters in directory.
        self.co_parameters = co_parameters # list of co-parameters

class FoliatedIntersection:
    """This class represents a foliated intersection"""
    def __init__(self, foliation1, foliation2, sampling_function):
        # check if the input is BaseFoliation class
        if not isinstance(foliation1, BaseFoliation):
            raise Exception("foliation1 is not a BaseFoliation class")
        if not isinstance(foliation2, BaseFoliation):
            raise Exception("foliation2 is not a BaseFoliation class")
        if not callable(sampling_function):
            raise Exception("sampling_function is not a function")

        self.foliation1 = foliation1
        self.foliation2 = foliation2
        # the sampling function will receive two list of co_parameters from each foliation, then return a BaseIntersection class.
        self.sampling_function = sampling_function

    def sample(self):
        """
            Sample a configuration from the foliated intersection.
            The sampling function will receive two list of co_parameters from each foliation, then return a success flag, co_parameter_index from first foliation, co_paramter_index from second foliation, and BaseIntersection class.
        """

        success_flag, co_parameter1_index, co_parameter2_index, sampled_intersection = self.sampling_function(self.foliation1.co_parameters, self.foliation2.co_parameters)

        if not isinstance(success_flag, bool):
            raise Exception("The first return value of sampling function is not a boolean value!!!")
        
        if not isinstance(co_parameter1_index, int):
            raise Exception("The second return value(the index of sampled co-parameter1) of sampling function is not an integer value!!!")

        if not isinstance(co_parameter2_index, int):
            raise Exception("The third return value(the index of sampled co-parameter2) of sampling function is not an integer value!!!")

        if not success_flag: # if the success flag is false, then return false and whatever.
            return success_flag, co_parameter1_index, co_parameter2_index, sampled_intersection

        if not isinstance(sampled_intersection, BaseIntersection):
            raise Exception("Sampled intersection is not a BaseIntersection class")

        return success_flag, co_parameter1_index, co_parameter2_index, sampled_intersection

class FoliatedProblem:
    def __init__(self, problem_name):
        """Constructor for FoliatedProblem class"""
        self.problem_name = problem_name

    def set_foliation_n_foliated_intersection(self, foliations, foliated_intersections):
        """Set foliations and foliated intersections to the experiment"""
        self.foliations = foliations
        self.foliated_intersections = foliated_intersections
        #TODO: Construct the problem into graph structure.

    def set_start_and_goal(self, start_configuration, start_foliation, start_co_parameter, goal_configuration, goal_foliation, goal_co_parameter):
        """Set the start and goal configurations"""
        self.start_configuration = start_configuration
        self.start_foliation = start_foliation
        self.start_co_parameter = start_co_parameter
        self.goal_configuration = goal_configuration
        self.goal_foliation = goal_foliation
        self.goal_co_parameter = goal_co_parameter

    def save(self):
        '''Save the foliated problem'''
        #TODO: Save the graph structure into a file.
        pass

    def load(self):
        '''Load the foliated problem'''
        #TODO: Load the graph structure from a file.
        pass
        