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
        raise NotImplementedError("Please Implement this method")

    # reset task planner
    def reset_task_planner(self):
        raise NotImplementedError("Please Implement this method")
        
    def add_manifold(self, manifold_constraint_, manifold_id_):
        raise NotImplementedError("Please Implement this method")

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_):
        """
        add intersection to the manifold
        """
        raise NotImplementedError("Please Implement this method")

    def set_start_and_goal(self,
                            start_manifold_id_,
                            start_configuration_, 
                            goal_manifold_id_,
                            goal_configuration_):
        """
        set start and goal configurations
        """
        raise NotImplementedError("Please Implement this method")

    def generate_task_sequence(self):
        """
        generate task sequence
        """
        raise NotImplementedError("Please Implement this method")

    def update(self, task_graph_info_, plan_):
        """
        update task planner
        """
        raise NotImplementedError("Please Implement this method")

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
        if plan_[0]:
            self.task_graph.edges[task_graph_info_]['weight'] += 0.01
        else:
            self.task_graph.edges[task_graph_info_]['weight'] += 1
            # find all similar task in the task graph and increase their weights.
            # TODO: implement this

class MDPTaskPlanner(BaseTaskPlanner):
    def __init__(self):
        # Constructor
        super(BaseTaskPlanner, self).__init__() # python 2
        # super().__init__() # python 3

    def reset_task_planner(self):
        self.task_graph = nx.Graph()
        self.manifold_constraints = {} # the constraints of each manifold
        self.manifold_intersections = {} # the intersections of each manifold
        self.new_intersection_id = 0
        self.gamma = 0.9

    def add_manifold(self, manifold_constraint_, manifold_id_):
        self.manifold_constraints[manifold_id_] = manifold_constraint_
        self.manifold_intersections[manifold_id_] = []

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_):
        self.task_graph.add_node(self.new_intersection_id, intersection=intersection_)
        for i in self.manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(i, self.new_intersection_id, probability=0.5, manifold_id=manifold_id1_)
        for i in self.manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(i, self.new_intersection_id, probability=0.5, manifold_id=manifold_id2_)
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
            self.task_graph.add_edge('start', i, probability=0.5, manifold_id=start_manifold_id_)

        for i in self.manifold_intersections[goal_manifold_id_]:
            self.task_graph.add_edge(i, 'goal', probability=0.5, manifold_id=goal_manifold_id_)

        # initialize the value function of each node
        self.value_function = {node: 0 for node in self.task_graph.nodes}
        self.value_function['goal'] = 1.0

    def generate_task_sequence(self):
        # perform value iteration
        for _ in range(100):
            new_value_function = {}
            # update the vlaue function for each node
            for node in self.task_graph.nodes:
                if node == 'goal': # goal node is the terminal node, so no need to update
                    new_value_function[node] = 1.0
                    continue

                # find the max expected value of current node
                value_of_all_neighbors = [
                    self.task_graph.edges[node, neighbor]['probability'] * (1.0 + self.gamma * self.value_function[neighbor]) - 1.0
                        for neighbor in self.task_graph.neighbors(node)
                ]
                new_value_function[node] = max(value_of_all_neighbors)

            # if the value function converges, then stop
            if np.all(np.isclose(list(self.value_function.values()), list(new_value_function.values()), rtol=1e-04, atol=1e-04)):
        			break
            self.value_function = new_value_function

        # find the shortest path from start to goal
        shortest_path = []
        current_node = 'start'
        while current_node != 'goal':
            shortest_path.append(current_node)
            current_node = max(self.task_graph.neighbors(current_node), key=lambda x: self.value_function[x])
        shortest_path.append('goal')

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
        if plan_[0]:
            # set the transition probability to 1 because the task is successfully completed
            self.task_graph.edges[task_graph_info_]['probability'] = 1.0
        else:
            self.task_graph.edges[task_graph_info_]['probability'] *= 0.5
            # find all similar task in the task graph and decrease their transition probabilities.
            #TODO: implement this