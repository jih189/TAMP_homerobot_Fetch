import numpy as np
import networkx as nx


class Task:
    def __init__(self, 
                constraint_, 
                start_configuration_,
                goal_configuration_,
                ):
        # Constructor
        self.constraint = constraint_
        self.start_configuration = start_configuration_
        self.goal_configuration = goal_configuration_
        self.distributions = []

    def add_gaussian_distribution(self, distribution_):
        self.distributions.append(distribution_)

    def clear_distributions(self):
        self.distributions = []

    def set_task_graph_info(self, task_graph_info_):
        self.task_graph_info = task_graph_info_

class GaussianDistribution:
    def __init__(self, 
                mean_,
                covariance_,
                ):
        # Constructor
        self.mean = mean_
        self.covariance = covariance_

class GMM:
    def __init__(self):
        # Constructor
        self.distributions = []
        self.edge_of_distribution = []

    def get_distribution(self, configuration_):
        # find which distribution the configuration belongs to
        # then return the distribution
        # TODO: implement this

        return GaussianDistribution(0, 0)

class BaseTaskPlanner(object):
    def __init__(self):
        # Constructor
        pass

    # reset task planner
    def reset_task_planner(self):
        pass
        
    def add_manifold(self, manifold_constraint_, manifold_id_):
        pass

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_):
        """
        add intersection to the manifold
        """
        pass

    def set_start_and_goal(self,
                            start_manifold_id_,
                            start_configuration_, 
                            goal_manifold_id_,
                            goal_configuration_):
        """
        set start and goal configurations
        """
        pass

    def generate_task_sequence(self):
        """
        generate task sequence
        """
        pass

    def update(self, task_graph_info_, plan_):
        """
        update task planner
        """
        pass

class MTGTaskPlanner(BaseTaskPlanner):
    def __init__(self):
        # Constructor
        super(BaseTaskPlanner, self).__init__() # python 2
        # super().__init__() # python 3

    def reset_task_planner(self):
        self.task_graph = nx.Graph()
        self.manifold_constraints = {} # the constraints of each manifold
        self.manifold_intersections = {} # the intersections of each manifold
        self.new_intersection_id = 0

    def add_manifold(self, manifold_constraint_, manifold_id_):
        self.manifold_constraints[manifold_id_] = manifold_constraint_
        self.manifold_intersections[manifold_id_] = []

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_):
        self.task_graph.add_node(self.new_intersection_id, intersection=intersection_)
        for i in self.manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(i, self.new_intersection_id, weight=0, manifold_id=manifold_id1_)
        for i in self.manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(i, self.new_intersection_id, weight=0, manifold_id=manifold_id2_)
        self.manifold_intersections[manifold_id1_].append(self.new_intersection_id)
        self.manifold_intersections[manifold_id2_].append(self.new_intersection_id)
        self.new_intersection_id += 1

    def set_start_and_goal(self,
                            start_manifold_id_,
                            start_configuration_, 
                            goal_manifold_id_,
                            goal_configuration_):
        # if start and goal are set, then remove them from the task graph
        if self.task_graph.has_node('start'):
            self.task_graph.remove_node('start')

        if self.task_graph.has_node('goal'):
            self.task_graph.remove_node('goal')

        # include start and goal configurations in the task graph
        self.task_graph.add_node('start', intersection = start_configuration_)
        self.task_graph.add_node('goal', intersection = goal_configuration_)

        for i in self.manifold_intersections[start_manifold_id_]:
            self.task_graph.add_edge('start', i, weight=0, manifold_id=start_manifold_id_)

        for i in self.manifold_intersections[goal_manifold_id_]:
            self.task_graph.add_edge(i, 'goal', weight=0, manifold_id=goal_manifold_id_)

    def generate_task_sequence(self):
        # find the shortest path from start to goal
        shortest_path = nx.shortest_path(self.task_graph, 'start', 'goal', weight='weight')
        task_sequence = []

        # construct the task sequence.
        for node1, node2 in zip(shortest_path[:-1], shortest_path[1:]):
            task = Task(self.manifold_constraints[self.task_graph.edges[node1, node2]['manifold_id']],
                        nx.get_node_attributes(self.task_graph, 'intersection')[node1],
                        nx.get_node_attributes(self.task_graph, 'intersection')[node2])
            
            task.set_task_graph_info((node1, node2))
            task_sequence.append(task)

        return task_sequence

    def update(self, task_graph_info_, plan_):
        if plan_.result:
            self.task_graph.edges[task_graph_info_]['weight'] += 0.01
        else:
            self.task_graph.edges[task_graph_info_]['weight'] += 1
            # find all similar task in the task graph and increase their weights.