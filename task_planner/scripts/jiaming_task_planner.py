import numpy as np
import networkx as nx

from sklearn import mixture


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
        self.edge_probabilities = []
        self._sklearn_gmm = None

    def get_distribution_index(self, configuration_):
        # find which distribution the configuration belongs to
        # then return the distribution
        # configuration_ is a (d,) element array : (d = 7)
        dist_num = self._sklearn_gmm.predict(configuration_.reshape((1, -1))).squeeze()
        return dist_num.item()

    def get_distribution(self, configuration_):
        # find which distribution the configuration belongs to
        # then return the distribution
        # configuration_ is a (d,) element array : (d = 7)
        dist_num = self._sklearn_gmm.predict(configuration_.reshape((1, -1))).squeeze()
        # return GaussianDistribution(self._sklearn_gmm.means_[dist_num], self._sklearn_gmm.covariances_[dist_num])
        return self.distributions[dist_num]

    def load_distributions(self, dir_name = "../gmm/"):


        means = np.load(dir_name + 'gmm_means.npy')
        covariances = np.load(dir_name + 'gmm_covariances.npy')

        # Create an sklearn Gaussian Mixture Model 
        self._sklearn_gmm = mixture.GaussianMixture(n_components = len(means), covariance_type='full')
        self._sklearn_gmm.precisions_cholesky_ = np.linalg.cholesky(np.linalg.inv(covariances))
        self._sklearn_gmm.weights_ = np.load(dir_name + 'gmm_weights.npy')
        self._sklearn_gmm.means_ = means
        self._sklearn_gmm.covariances_ = covariances

        for mean, covariance in zip(means, covariances):
            self.distributions.append(GaussianDistribution(mean, covariance))

        self.edge_of_distribution = np.load(dir_name + 'edges.npy')
        self.edge_probabilities = np.load(dir_name + 'edge_probabilities.npy')

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
        both start and goal configurations are intersection here.
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
        self.task_graph = nx.DiGraph()
        self.manifold_constraints = {} # the constraints of each manifold
        self.incomming_manifold_intersections = {} # the incomming intersections of each manifold
        self.outgoing_manifold_intersections = {} # the outgoing intersections of each manifold
        self.new_intersection_id = 0

    def add_manifold(self, manifold_constraint_, manifold_id_):
        self.manifold_constraints[manifold_id_] = manifold_constraint_
        self.incomming_manifold_intersections[manifold_id_] = []
        self.outgoing_manifold_intersections[manifold_id_] = []

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_):
        intersection_from_1_to_2_id = self.new_intersection_id
        self.new_intersection_id += 1
        intersection_from_2_to_1_id = self.new_intersection_id
        self.new_intersection_id += 1

        # add node for intersection from manifold 1 to manifold 2
        self.task_graph.add_node(intersection_from_1_to_2_id, intersection=intersection_)

        inversed_intersection = (intersection_[0], intersection_[1][::-1], intersection_[2], intersection_[3], intersection_[4])
        # add node for intersection from manifold 2 to manifold 1
        self.task_graph.add_node(intersection_from_2_to_1_id, intersection=inversed_intersection) # here

        for i in self.incomming_manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(i, intersection_from_1_to_2_id, weight=0, manifold_id=manifold_id1_)
        for i in self.outgoing_manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(intersection_from_1_to_2_id, i, weight=0, manifold_id=manifold_id2_)

        for i in self.outgoing_manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(intersection_from_2_to_1_id, i, weight=0, manifold_id=manifold_id1_)
        for i in self.incomming_manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(i, intersection_from_2_to_1_id, weight=0, manifold_id=manifold_id2_)

        self.outgoing_manifold_intersections[manifold_id1_].append(intersection_from_1_to_2_id)
        self.incomming_manifold_intersections[manifold_id2_].append(intersection_from_1_to_2_id)

        self.outgoing_manifold_intersections[manifold_id2_].append(intersection_from_2_to_1_id)
        self.incomming_manifold_intersections[manifold_id1_].append(intersection_from_2_to_1_id)

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

        for i in self.outgoing_manifold_intersections[start_manifold_id_]:
            self.task_graph.add_edge('start', i, weight=0, manifold_id=start_manifold_id_)

        for i in self.incomming_manifold_intersections[goal_manifold_id_]:
            self.task_graph.add_edge(i, 'goal', weight=0, manifold_id=goal_manifold_id_)

    def generate_task_sequence(self):
        # check the connectivity of the task graph from start to goal
        if not nx.has_path(self.task_graph, 'start', 'goal'):
            return []

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
        self.task_graph = nx.DiGraph()
        self.manifold_constraints = {} # the constraints of each manifold
        self.incomming_manifold_intersections = {} # the incomming intersections of each manifold
        self.outgoing_manifold_intersections = {} # the outgoing intersections of each manifold
        self.new_intersection_id = 0
        self.gamma = 0.9

    def add_manifold(self, manifold_constraint_, manifold_id_):
        self.manifold_constraints[manifold_id_] = manifold_constraint_
        self.incomming_manifold_intersections[manifold_id_] = []
        self.outgoing_manifold_intersections[manifold_id_] = []

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_):
        intersection_from_1_to_2_id = self.new_intersection_id
        self.new_intersection_id += 1
        intersection_from_2_to_1_id = self.new_intersection_id
        self.new_intersection_id += 1

        # add node for intersection from manifold 1 to manifold 2
        self.task_graph.add_node(intersection_from_1_to_2_id, intersection=intersection_)

        inversed_intersection = (intersection_[0], intersection_[1][::-1], intersection_[2], intersection_[3], intersection_[4])
        # add node for intersection from manifold 2 to manifold 1
        self.task_graph.add_node(intersection_from_2_to_1_id, intersection=inversed_intersection) # here

        for i in self.incomming_manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(i, intersection_from_1_to_2_id, probability=0, manifold_id=manifold_id1_)
        for i in self.outgoing_manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(intersection_from_1_to_2_id, i, probability=0, manifold_id=manifold_id2_)

        for i in self.outgoing_manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(intersection_from_2_to_1_id, i, probability=0, manifold_id=manifold_id1_)
        for i in self.incomming_manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(i, intersection_from_2_to_1_id, probability=0, manifold_id=manifold_id2_)

        self.outgoing_manifold_intersections[manifold_id1_].append(intersection_from_1_to_2_id)
        self.incomming_manifold_intersections[manifold_id2_].append(intersection_from_1_to_2_id)

        self.outgoing_manifold_intersections[manifold_id2_].append(intersection_from_2_to_1_id)
        self.incomming_manifold_intersections[manifold_id1_].append(intersection_from_2_to_1_id)

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

        for i in self.outgoing_manifold_intersections[start_manifold_id_]:
            self.task_graph.add_edge('start', i, probability=0.5, manifold_id=start_manifold_id_)

        for i in self.incomming_manifold_intersections[goal_manifold_id_]:
            self.task_graph.add_edge(i, 'goal', probability=0.5, manifold_id=goal_manifold_id_)

        # initialize the value function of each node
        self.value_function = {node: 0 for node in self.task_graph.nodes}
        self.value_function['goal'] = 1.0

    def generate_task_sequence(self):
        # check the connectivity of the task graph from start to goal
        if not nx.has_path(self.task_graph, 'start', 'goal'):
            return []

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

class MTGTaskPlannerWithGMM(BaseTaskPlanner):
    def __init__(self):
        # Constructor
        super(BaseTaskPlanner, self).__init__()
        # super().__init__() # python 3

    def reset_task_planner(self, gmm):
        self.task_graph = nx.DiGraph()
        self.manifold_constraints = {} # the constraints of each manifold
        self.gmm_ = gmm

    def add_manifold(self, manifold_constraint_, manifold_id_):
        self.manifold_constraints[manifold_id_] = manifold_constraint_

        # construct a set of nodes represented by a pair (foliation id, manifold id, GMM id)
        for i in range(len(self.gmm_.distributions)):
            self.task_graph.add_node((manifold_id_[0], manifold_id_[1], i), weight = 0.0)

        for edge in self.gmm_.edge_of_distribution:
            self.task_graph.add_edge(
                (manifold_id_[0], manifold_id_[1], edge[0]), 
                (manifold_id_[0], manifold_id_[1], edge[1]), 
                has_intersection=False, 
                intersection=None
            )

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_):
        # connect two distribution of this intersection_ between two different manifolds(manifold1 and manifold2) if they have the same ditribution id in GMM.
        # first, find the related distribution that the intersection's ends are in in different manifolds.

        # as a reminder, intersection_ is a tuple of 
        # (has object in hand, 
        #  trajectory motion, 
        #  in hand pose,
        #  object mesh,
        #  object name)

        # get the first of configuration of the intersection_
        configuration1 = intersection_[1][0]
        distribution_id_in_manifold1 = self.gmm_.get_distribution_index(configuration1)

        # get the second of configuration of the intersection_
        configuration2 = intersection_[1][-1]
        distribution_id_in_manifold2 = self.gmm_.get_distribution_index(configuration2)

        if(not self.task_graph.has_edge((manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1), (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2)) and 
           not self.task_graph.has_edge((manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2), (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1))
        ):
            self.task_graph.add_edge(
                (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1), 
                (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2), 
                has_intersection=True, 
                intersection=intersection_
            )

            inversed_intersection = (intersection_[0], intersection_[1][::-1], intersection_[2], intersection_[3], intersection_[4])
            
            self.task_graph.add_edge(
                (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2),
                (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1),
                has_intersection=True,
                intersection=inversed_intersection
            )

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
        self.task_graph.add_node('start', weight = 0.0)
        self.task_graph.add_node('goal', weight = 0.0)

        self.task_graph.add_edge(
            'start', 
            (start_manifold_id_[0], start_manifold_id_[1], self.gmm_.get_distribution_index(np.array(start_configuration_[1][0]))), 
            has_intersection=False, 
            intersection=None
        )

        self.task_graph.add_edge(
            (goal_manifold_id_[0], goal_manifold_id_[1], self.gmm_.get_distribution_index(np.array(goal_configuration_[1][0]))), 
            'goal', 
            has_intersection=True, 
            intersection=goal_configuration_
        )

        self.current_start_configuration = start_configuration_

    def generate_task_sequence(self):
        # find the shortest path from start to goal
        shortest_path = nx.shortest_path(self.task_graph, 'start', 'goal', weight='weight')

        task_sequence = []
        task_start_configuration = self.current_start_configuration
        task_gaussian_distribution = []

        # construct the task sequence.
        for node1, node2 in zip(shortest_path[:-1], shortest_path[1:]):
            if self.task_graph.get_edge_data(node1, node2)['has_intersection']:
                task = Task(
                    self.manifold_constraints[(node1[0], node1[1])],
                    task_start_configuration,
                    self.task_graph.get_edge_data(node1, node2)['intersection']
                )

                task.distributions = list(task_gaussian_distribution)
                task.set_task_graph_info(node1[0]) # we use the manifold id as task graph information here
                task_sequence.append(task)

                # ready for the next task.
                if node2 != 'goal': # if the edge is to goal, then no need to prepare for the next task
                    task_gaussian_distribution = [self.gmm_.distributions[node2[1]]]
                    task_start_configuration = self.task_graph.get_edge_data(node1, node2)['intersection']
            else:
                # edge in the same manifold except start and goal transition
                task_gaussian_distribution.append(self.gmm_.distributions[node2[1]])
        return task_sequence

    def update(self, task_graph_info_, plan_):
        # use the sample data to update the task graph.
        # TODO: implement this later


        if plan_[0]:
            # increase the value of all distribution in the same manifold with a small value.
            for g_id in range(len(self.gmm_.distributions)):
                self.task_graph.nodes[(task_graph_info_[0], task_graph_info_[1], g_id)]['weight'] += 0.01
        else:
            # increase the value of all distribution in the same manifold with a large value.
            for g_id in range(len(self.gmm_.distributions)):
                self.task_graph.nodes[(task_graph_info_[0], task_graph_info_[1], g_id)]['weight'] += 1.0
