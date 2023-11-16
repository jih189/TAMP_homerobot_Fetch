#!/usr/bin/env python
# from experiment_scripts.experiment_helper import Experiment, Manifold, Intersection
import os
import json

# user needs to implement this function
class BaseIntersection(object):
    """
        This class represents a base intersection.
        It is an abstract class. It is used to represent the intersection of two manifolds.
        User needs to implement both get and inverse functions.
    """
    def set_foliation_names_and_co_parameter_indexes(self, foliation1_name, co_parameter1_index, foliation2_name, co_parameter2_index):
        """Set the foliation and co-parameter index"""
        self.foliation1_name = foliation1_name
        self.co_parameter1_index = co_parameter1_index
        self.foliation2_name = foliation2_name
        self.co_parameter2_index = co_parameter2_index

    def get_foliation_names_and_co_parameter_indexes(self):
        """Get the foliation and co-parameter index"""
        return self.foliation1_name, self.co_parameter1_index, self.foliation2_name, self.co_parameter2_index

    def inverse(self):
        """Return the inverse of the intersection, user needs to implement this function"""
        """Inverse function does not inverse the foliation and co-parameter index."""
        # Return inversed the intersection
        raise NotImplementedError("Please Implement this method")

    def save(self, file_path):
        """Save the intersection"""
        raise NotImplementedError("Please Implement this method")

    @staticmethod
    def load(file_path):
        """Load the intersection"""
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

    def save(self, file_path):
        """Save the foliation"""
        raise NotImplementedError("Please Implement this method")

    @staticmethod
    def load(file_path):
        """Load the foliation and return a foliation object"""
        raise NotImplementedError("Please Implement this method")

class FoliatedIntersection:
    """This class represents a foliated intersection"""
    def __init__(self, foliation1, foliation2, sampling_function, prepare_sample_function=None, sample_done_function=None):
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
        self.prepare_sample_function = prepare_sample_function
        self.sampling_function = sampling_function
        self.sample_done_function = sample_done_function

    def prepare_sample(self):
        if self.prepare_sample_function is None:
            return
        self.prepare_sample_function()

    def sample_done(self):
        if self.sample_done_function is None:
            return
        self.sample_done_function()

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
        self.has_setup = False
        self.foliations = []
        self.intersections = [] # list of intersections.
        self.foliated_intersections = []

    def set_foliation_n_foliated_intersection(self, foliations, foliated_intersections):
        """Set foliations and foliated intersections to the experiment"""
        self.foliations = foliations
        self.foliated_intersections = foliated_intersections
        #TODO: Construct the problem into graph structure.

        self.has_setup = True

    def sample_intersections(self):

        if not self.has_setup:
            raise Exception("The foliated problem has not been setup!!!")

        for foliated_intersection in self.foliated_intersections:
            # print "sample between " + foliated_intersection.foliation1.foliation_name + " and " + foliated_intersection.foliation2.foliation_name
            foliated_intersection.prepare_sample()

            for i in range(0, 10):
                success_flag, co_parameter1_index, co_parameter2_index, sampled_intersection = foliated_intersection.sample()

                if success_flag:
                    # print 'co_parameter1_index: ' + str(co_parameter1_index) + ', co_parameter2_index: ' + str(co_parameter2_index)

                    sampled_intersection.set_foliation_names_and_co_parameter_indexes(
                        foliated_intersection.foliation1.foliation_name,
                        co_parameter1_index,
                        foliated_intersection.foliation2.foliation_name,
                        co_parameter2_index
                    )
                    self.intersections.append(sampled_intersection)

                    # inverse the intersection
                    inversed_sampled_intersection = sampled_intersection.inverse()
                    inversed_sampled_intersection.set_foliation_names_and_co_parameter_indexes(
                        foliated_intersection.foliation2.foliation_name,
                        co_parameter2_index,
                        foliated_intersection.foliation1.foliation_name,
                        co_parameter1_index
                    )

                    # append the inverse intersection
                    self.intersections.append(inversed_sampled_intersection)
                
            foliated_intersection.sample_done()

    # def set_start_and_goal(self, start_configuration, start_foliation, start_co_parameter, goal_configuration, goal_foliation, goal_co_parameter):
    #     """Set the start and goal configurations"""
    #     self.start_configuration = start_configuration
    #     self.start_foliation = start_foliation
    #     self.start_co_parameter = start_co_parameter
    #     self.goal_configuration = goal_configuration
    #     self.goal_foliation = goal_foliation
    #     self.goal_co_parameter = goal_co_parameter

    def save(self, dir_name):
        '''Save the foliated problem'''
        
        if not self.has_setup:
            raise Exception("The foliated problem has not been setup!!!")
        
        if os.path.exists(dir_name):
            # delete the directory
            os.system("rm -rf " + dir_name)
        
        os.makedirs(dir_name)
        problem_data ={
            "problem_name": self.problem_name,
            "foliations": [],
            "intersections": []
        }

        # create a directory for foliations
        os.makedirs(dir_name + "/foliations")

        for foliation in self.foliations:
            foliation.save(dir_name + '/foliations')
            problem_data["foliations"].append(foliation.foliation_name)

        # create a directory for intersections
        os.makedirs(dir_name + "/intersections")

        for i, intersection in enumerate(self.intersections):
            intersection.save(dir_name + "/intersections/intersection_" + str(i) + ".json")
            problem_data["intersections"].append("intersection_" + str(i))

        # save the problem data
        with open(dir_name + "/problem_data.json", "w") as f:
            json.dump(problem_data, f, indent=4)

    @staticmethod
    def load(foliation_class, intersection_class, dir_name):
        '''Load the foliated problem'''

        # check if foliation_class is a subclass of BaseFoliation
        if not issubclass(foliation_class, BaseFoliation):
            raise Exception("foliation_class is not a subclass of BaseFoliation!!!")
        
        # check if intersection_class is a subclass of BaseIntersection
        if not issubclass(intersection_class, BaseIntersection):
            raise Exception("intersection_class is not a subclass of BaseIntersection!!!")

        # check if dir_name exists
        if not os.path.exists(dir_name):
            raise Exception("The directory does not exist!!!")

        # check if problem_data.json exists
        if not os.path.exists(dir_name + "/problem_data.json"):
            raise Exception("The problem_data.json does not exist!!!")

        # check if foliations directory exists
        if not os.path.exists(dir_name + "/foliations"):
            raise Exception("The foliations directory does not exist!!!")

        # load the problem data
        with open(dir_name + "/problem_data.json", "r") as f:
            problem_data = json.load(f)

        loaded_problem = FoliatedProblem(problem_data["problem_name"])

        loaded_problem.set_foliation_n_foliated_intersection(
            [foliation_class.load(dir_name + "/foliations/" + foliation_name + ".json") for foliation_name in problem_data["foliations"]], 
            [intersection_class.load(dir_name + "/intersections/" + intersection_name + ".json") for intersection_name in problem_data["intersections"]]
        )

        return loaded_problem

    # def get_number_of_intersections(self):
    #     '''Return the number of intersections'''
        