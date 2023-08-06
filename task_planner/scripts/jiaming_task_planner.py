import numpy as np
import networkx as nx

from sklearn import mixture

class ManifoldDetail:
    '''
    ManifoldDetail contains the detail of a manifold.
    '''
    def __init__(self,
                constraint_,
                has_object_in_hand_,
                object_pose_,
                object_mesh_,
                object_name_,
                ):
        # Constructor
        self.constraint = constraint_
        self.has_object_in_hand = has_object_in_hand_
        self.object_pose = object_pose_
        self.object_mesh = object_mesh_
        self.object_name = object_name_

    def print_manifold_detail(self):
        print("----------- manifold:")
        print("constraint:")
        print("moveit constraint:")
        print(self.constraint)
        print("has_object_in_hand:")
        print(self.has_object_in_hand)
        print("object_pose:")
        print(self.object_pose)
        print("object_mesh:")
        print(self.object_mesh)
        print("object_name:")
        print(self.object_name)
        print("-----------")

class IntersectionDetail:
    '''
    IntersectionDetail contains the detail of an intersection.
    '''
    def __init__(self,
                has_object_in_hand_,
                trajectory_motion_,
                in_hand_pose_,
                object_mesh_,
                object_name_,
                ):
        # Constructor
        self.has_object_in_hand = has_object_in_hand_
        self.trajectory_motion = trajectory_motion_
        self.in_hand_pose = in_hand_pose_
        self.object_mesh = object_mesh_
        self.object_name = object_name_

    def get_inverse_motion(self):
        return IntersectionDetail(
            self.has_object_in_hand,
            self.trajectory_motion[::-1],
            self.in_hand_pose,
            self.object_mesh,
            self.object_name
        )

class Task:
    def __init__(self, 
                manifold_detail_, 
                start_configuration_,
                goal_configuration_,
                next_motion_
                ):
        # Constructor
        self.manifold_detail = manifold_detail_
        self.start_configuration = start_configuration_
        self.goal_configuration = goal_configuration_
        self.next_motion = next_motion_ # the robot motion after the task is completed
        self.distributions = []

    def add_gaussian_distribution(self, distribution_):
        self.distributions.append(distribution_)

    def clear_distributions(self):
        self.distributions = []

    def set_task_graph_info(self, task_graph_info_):
        self.task_graph_info = task_graph_info_

    def print_task_detail(self):
        print("----------- task:")
        print("start configuration:")
        print(self.start_configuration)
        print("goal configuration:")
        print(self.goal_configuration)
        print("manifold detail:")
        self.manifold_detail.print_manifold_detail()
        print("task graph info:")
        print(self.task_graph_info)
        print("-----------")

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

        means = np.load(dir_name + 'means.npy')
        covariances = np.load(dir_name + 'covariances.npy')

        # Create an sklearn Gaussian Mixture Model 
        self._sklearn_gmm = mixture.GaussianMixture(n_components = len(means), covariance_type='full')
        self._sklearn_gmm.precisions_cholesky_ = np.linalg.cholesky(np.linalg.inv(covariances))
        self._sklearn_gmm.weights_ = np.load(dir_name + 'weights.npy')
        self._sklearn_gmm.means_ = means
        self._sklearn_gmm.covariances_ = covariances

        for mean, covariance in zip(means, covariances):
            self.distributions.append(GaussianDistribution(mean, covariance))
        print("Loaded %d distributions " % len(means), dir_name)
        self.edge_of_distribution = np.load(dir_name + 'edges.npy')
        self.edge_probabilities = np.load(dir_name + 'edge_probabilities.npy')

class BaseTaskPlanner(object):
    def __init__(self):
        # Constructor
        raise NotImplementedError("Please Implement this method")

    # reset task planner
    def reset_task_planner(self):
        raise NotImplementedError("Please Implement this method")
        
    def add_manifold(self, manifold_info_, manifold_id_):
        raise NotImplementedError("Please Implement this method")

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_detail_):
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
        self.planner_name = "MTGTaskPlanner"

    def reset_task_planner(self):
        self.task_graph = nx.DiGraph()
        self.manifold_info = {} # the detail of each manifold
        self.incomming_manifold_intersections = {} # the incomming intersections of each manifold
        self.outgoing_manifold_intersections = {} # the outgoing intersections of each manifold
        self.new_intersection_id = 0

    def add_manifold(self, manifold_info_, manifold_id_):
        self.manifold_info[manifold_id_] = manifold_info_

        self.incomming_manifold_intersections[manifold_id_] = []
        self.outgoing_manifold_intersections[manifold_id_] = []

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_detail_):
        intersection_from_1_to_2_id = self.new_intersection_id
        self.new_intersection_id += 1
        intersection_from_2_to_1_id = self.new_intersection_id
        self.new_intersection_id += 1

        # add node for intersection from manifold 1 to manifold 2
        self.task_graph.add_node(intersection_from_1_to_2_id, intersection=intersection_detail_)

        # add node for intersection from manifold 2 to manifold 1
        self.task_graph.add_node(intersection_from_2_to_1_id, intersection=intersection_detail_.get_inverse_motion())

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

        self.task_graph.add_node(
            'start', 
            intersection = IntersectionDetail(
                False, 
                [start_configuration_], 
                None,
                None,
                None
            )
        )
        self.task_graph.add_node(
            'goal', 
            intersection = IntersectionDetail(
                False,
                [goal_configuration_],
                None,
                None,
                None
            )
        )

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
            task = Task(
                        self.manifold_info[self.task_graph.edges[node1, node2]['manifold_id']],
                        nx.get_node_attributes(self.task_graph, 'intersection')[node1].trajectory_motion[-1],
                        nx.get_node_attributes(self.task_graph, 'intersection')[node2].trajectory_motion[0],
                        nx.get_node_attributes(self.task_graph, 'intersection')[node2].trajectory_motion
                    )
            
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

        self.planner_name = "MDPTaskPlanner"

    def reset_task_planner(self):
        self.task_graph = nx.DiGraph()
        self.manifold_info = {} # the constraints of each manifold
        self.incomming_manifold_intersections = {} # the incomming intersections of each manifold
        self.outgoing_manifold_intersections = {} # the outgoing intersections of each manifold
        self.new_intersection_id = 0
        self.gamma = 0.9

    def add_manifold(self, manifold_info_, manifold_id_):
        self.manifold_info[manifold_id_] = manifold_info_

        self.incomming_manifold_intersections[manifold_id_] = []
        self.outgoing_manifold_intersections[manifold_id_] = []

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_detail_):
        intersection_from_1_to_2_id = self.new_intersection_id
        self.new_intersection_id += 1
        intersection_from_2_to_1_id = self.new_intersection_id
        self.new_intersection_id += 1

        # add node for intersection from manifold 1 to manifold 2
        self.task_graph.add_node(intersection_from_1_to_2_id, intersection=intersection_detail_)

        # add node for intersection from manifold 2 to manifold 1
        self.task_graph.add_node(intersection_from_2_to_1_id, intersection=intersection_detail_.get_inverse_motion())

        for i in self.incomming_manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(i, intersection_from_1_to_2_id, probability=0.5, manifold_id=manifold_id1_)
        for i in self.outgoing_manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(intersection_from_1_to_2_id, i, probability=0.5, manifold_id=manifold_id2_)

        for i in self.outgoing_manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(intersection_from_2_to_1_id, i, probability=0.5, manifold_id=manifold_id1_)
        for i in self.incomming_manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(i, intersection_from_2_to_1_id, probability=0.5, manifold_id=manifold_id2_)

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
        self.task_graph.add_node(
            'start', 
            intersection = IntersectionDetail(
                False, 
                [start_configuration_], 
                None,
                None,
                None
            )
        )
        self.task_graph.add_node(
            'goal', 
            intersection = IntersectionDetail(
                False,
                [goal_configuration_],
                None,
                None,
                None
            )
        )

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
            # update the value function for each node
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
            task = Task(
                        self.manifold_info[self.task_graph.edges[node1, node2]['manifold_id']],
                        nx.get_node_attributes(self.task_graph, 'intersection')[node1].trajectory_motion[-1],
                        nx.get_node_attributes(self.task_graph, 'intersection')[node2].trajectory_motion[0],
                        nx.get_node_attributes(self.task_graph, 'intersection')[node2].trajectory_motion
                    )
            
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
    def __init__(self, gmm):
        # Constructor
        super(BaseTaskPlanner, self).__init__()
        # super().__init__() # python 3

        self.gmm_ = gmm

        self.planner_name = "MTGTaskPlannerWithGMM"

    def reset_task_planner(self):
        self.task_graph = nx.DiGraph()
        self.manifold_info = {} # the constraints of each manifold

    def add_manifold(self, manifold_info_, manifold_id_):
        self.manifold_info[manifold_id_] = manifold_info_

        # construct a set of nodes represented by a tuple (foliation id, manifold id, GMM id)
        for i in range(len(self.gmm_.distributions)):
            self.task_graph.add_node((manifold_id_[0], manifold_id_[1], i), weight = 0.0)

        for edge in self.gmm_.edge_of_distribution:
            self.task_graph.add_edge(
                (manifold_id_[0], manifold_id_[1], edge[0]), 
                (manifold_id_[0], manifold_id_[1], edge[1]), 
                has_intersection=False, 
                intersection=None
            )

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_detail_):
        # connect two distribution of this intersection_detail_ between two different manifolds(manifold1 and manifold2) if they have the same ditribution id in GMM.
        # first, find the related distribution that the intersection's ends are in in different manifolds.

        # get the first of configuration of the intersection_detail
        configuration1 = intersection_detail_.trajectory_motion[0]
        distribution_id_in_manifold1 = self.gmm_.get_distribution_index(configuration1)

        # get the second of configuration of the intersection_detail
        configuration2 = intersection_detail_.trajectory_motion[-1]
        distribution_id_in_manifold2 = self.gmm_.get_distribution_index(configuration2)

        if(not self.task_graph.has_edge((manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1), (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2)) and 
           not self.task_graph.has_edge((manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2), (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1))
        ):
            self.task_graph.add_edge(
                (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1), 
                (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2), 
                has_intersection=True, 
                intersection=intersection_detail_
            )

            self.task_graph.add_edge(
                (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2),
                (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1),
                has_intersection=True,
                intersection=intersection_detail_.get_inverse_motion()
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
            (
                start_manifold_id_[0], 
                start_manifold_id_[1], 
                self.gmm_.get_distribution_index(np.array(start_configuration_))
            ), 
            has_intersection=False, 
            intersection=None
        )

        self.task_graph.add_edge(
            (
                goal_manifold_id_[0], 
                goal_manifold_id_[1], 
                self.gmm_.get_distribution_index(np.array(goal_configuration_))
            ), 
            'goal', 
            has_intersection=True, 
            intersection=IntersectionDetail(
                False,
                [goal_configuration_],
                None,
                None,
                None
            )
        )

        self.current_start_configuration = start_configuration_

    def generate_task_sequence(self):
        # check the connectivity of the task graph from start to goal
        if not nx.has_path(self.task_graph, 'start', 'goal'):
            return []

        # find the shortest path from start to goal
        shortest_path = nx.shortest_path(self.task_graph, 'start', 'goal', weight='weight')

        task_sequence = []
        task_start_configuration = self.current_start_configuration
        task_gaussian_distribution = []

        # construct the task sequence.
        for node1, node2 in zip(shortest_path[:-1], shortest_path[1:]):
            if self.task_graph.get_edge_data(node1, node2)['has_intersection']:
                # current edge is a transition from one manifold to another manifold
                task = Task(
                    self.manifold_info[(node1[0], node1[1])],
                    task_start_configuration,
                    self.task_graph.get_edge_data(node1, node2)['intersection'].trajectory_motion[0],
                    self.task_graph.get_edge_data(node1, node2)['intersection'].trajectory_motion
                )

                task.distributions = list(task_gaussian_distribution)
                task.set_task_graph_info((node1[0], node1[1])) # we use the manifold id as task graph information here
                task_sequence.append(task)

                # ready for the next task.
                if node2 != 'goal': # if the edge is to goal, then no need to prepare for the next task
                    task_gaussian_distribution = [self.gmm_.distributions[node2[2]]]
                    task_start_configuration = self.task_graph.get_edge_data(node1, node2)['intersection'].trajectory_motion[-1]
            else:
                # edge in the same manifold except start and goal transition
                task_gaussian_distribution.append(self.gmm_.distributions[node2[2]])
        return task_sequence

    def update(self, task_graph_info_, plan_):
        # use the sample data to update the task graph.
        # sampled_state_tag hint
        # 0: collision free
        # 1: arm-env collision or out of joint limit
        # 2: path constraint violation
        # 3: infeasble state, you should ignore this
        # 4: obj-env collision

        sampled_data_numpy = np.array([sampled_data.sampled_state for sampled_data in plan_[4].verified_motions])
        sampled_data_distribution_id = self.gmm_._sklearn_gmm.predict(sampled_data_numpy).tolist()

        for i in range(len(sampled_data_distribution_id)):
            sampled_data_gmm_id = sampled_data_distribution_id[i]
            sampled_data_tag = plan_[4].verified_motions[i].sampled_state_tag

            if sampled_data_tag == 1:
                # increase the value of all distribution with the same mean and covariance
                for manifold_id in self.manifold_info.keys():
                    self.task_graph.nodes[(manifold_id[0], manifold_id[1], sampled_data_gmm_id)]['weight'] += 0.5

            elif sampled_data_tag == 2:
                self.task_graph.nodes[(task_graph_info_[0], task_graph_info_[1], sampled_data_gmm_id)]['weight'] += 0.3

            elif sampled_data_tag == 4:
                self.task_graph.nodes[(task_graph_info_[0], task_graph_info_[1], sampled_data_gmm_id)]['weight'] += 0.3

        if plan_[0]:
            # increase the value of all distribution in the same manifold with a small value.
            for g_id in range(len(self.gmm_.distributions)):
                self.task_graph.nodes[(task_graph_info_[0], task_graph_info_[1], g_id)]['weight'] += 0.01
        else:
            # increase the value of all distribution in the same manifold with a large value.
            for g_id in range(len(self.gmm_.distributions)):
                self.task_graph.nodes[(task_graph_info_[0], task_graph_info_[1], g_id)]['weight'] += 1.0

class MDPTaskPlannerWithGMM(BaseTaskPlanner):
    def __init__(self, gmm):
        # Init the constructor
        super(BaseTaskPlanner, self).__init__()

        self.gmm_ = gmm
        self.planner_name = "MDPTaskPlannerWithGMM"

    def reset_task_planner(self):
        self.task_graph = nx.DiGraph()
        self.manifold_info = {} # the constraints of each manifold
        self.gamma = 0.9
        self.value_iteration_iters = 100

    def add_manifold(self, manifold_info_, manifold_id_):
        self.manifold_info[manifold_id_] = manifold_info_

        # construct a set of nodes represented by a tuple (foliation id, manifold id, GMM id)
        for i in range(len(self.gmm_.distributions)):
            self.task_graph.add_node((manifold_id_[0], manifold_id_[1], i))

        for edge in self.gmm_.edge_of_distribution:
            self.task_graph.add_edge(
                (manifold_id_[0], manifold_id_[1], edge[0]), 
                (manifold_id_[0], manifold_id_[1], edge[1]), 
                has_intersection=False,
                intersection=None,
                probability = 0.5,
            )

    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_detail_):
        # connect two distribution of this intersection_detail_ between two different manifolds(manifold1 and manifold2) if they have the same ditribution id in GMM.
        # first, find the related distribution that the intersection's ends are in in different manifolds.

        # get the first of configuration of the intersection_detail
        configuration1 = intersection_detail_.trajectory_motion[0]
        distribution_id_in_manifold1 = self.gmm_.get_distribution_index(configuration1)

        # get the second of configuration of the intersection_detail
        configuration2 = intersection_detail_.trajectory_motion[-1]
        distribution_id_in_manifold2 = self.gmm_.get_distribution_index(configuration2)

        if(not self.task_graph.has_edge((manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1), (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2)) and 
           not self.task_graph.has_edge((manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2), (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1))
        ):
            self.task_graph.add_edge(
                (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1), 
                (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2), 
                has_intersection=True, 
                intersection=intersection_detail_,
                probability = 0.5,
            )

            self.task_graph.add_edge(
                (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2),
                (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1),
                has_intersection=True,
                intersection=intersection_detail_.get_inverse_motion(),
                probability = 0.5,
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
            (
                start_manifold_id_[0], 
                start_manifold_id_[1], 
                self.gmm_.get_distribution_index(np.array(start_configuration_))
            ), 
            has_intersection=False, 
            intersection=None,
            probability = 0.5
        )

        self.task_graph.add_edge(
            (
                goal_manifold_id_[0], 
                goal_manifold_id_[1], 
                self.gmm_.get_distribution_index(np.array(goal_configuration_))
            ), 
            'goal', 
            has_intersection=True, 
            intersection=IntersectionDetail(
                False,
                [goal_configuration_],
                None,
                None,
                None
            ),
            probability = 0.5
        )

        self.current_start_configuration = start_configuration_
        # initialize the value function of each node
        self.value_function = {node: 0 for node in self.task_graph.nodes}
        self.value_function['goal'] = 1.0

    def generate_task_sequence(self):
        # check the connectivity of the task graph from start to goal
        if not nx.has_path(self.task_graph, 'start', 'goal'):
            return []

        # need to generate shortest path first

        for _ in range(self.value_iteration_iters):
            new_value_function = {}

            #update the value function for each node
            for node in self.task_graph.nodes:
                if node == 'goal': 
                    new_value_function[node] = 1.0
                    continue

                # find the value of the current node
                value_of_all_neighbors = [
                    self.task_graph.edges[node, neighbor]['probability'] * (1.0 + self.gamma * self.value_function[neighbor]) - 1.0 for neighbor in self.task_graph.neighbors(node)
                ]

                # Update the value function of node with max of neighbors
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
        task_start_configuration = self.current_start_configuration
        task_gaussian_distribution = []

        # construct the task sequence.
        for node1, node2 in zip(shortest_path[:-1], shortest_path[1:]):
            if self.task_graph.get_edge_data(node1, node2)['has_intersection']:
                # current edge is a transition from one manifold to another manifold
                task = Task(
                    self.manifold_info[(node1[0], node1[1])],
                    task_start_configuration,
                    self.task_graph.get_edge_data(node1, node2)['intersection'].trajectory_motion[0],
                    self.task_graph.get_edge_data(node1, node2)['intersection'].trajectory_motion
                )

                task.distributions = list(task_gaussian_distribution)
                task.set_task_graph_info((node1[0], node1[1])) # we use the manifold id as task graph information here
                task_sequence.append(task)

                # ready for the next task.
                if node2 != 'goal': # if the edge is to goal, then no need to prepare for the next task
                    task_gaussian_distribution = [self.gmm_.distributions[node2[2]]]
                    task_start_configuration = self.task_graph.get_edge_data(node1, node2)['intersection'].trajectory_motion[-1]
            else:
                # edge in the same manifold except start and goal transition
                task_gaussian_distribution.append(self.gmm_.distributions[node2[2]])
        return task_sequence

    def update(self, task_graph_info_, plan_):


        # use the sample data to update the task graph.
        # sampled_state_tag hint
        # 0: collision free
        # 1: arm-env collision or out of joint limit
        # 2: path constraint violation
        # 3: infeasble state, you should ignore this
        # 4: obj-env collision
        # TODO () update fn - do not really understand task_graph_info_


        sampled_data_numpy = np.array([sampled_data.sampled_state for sampled_data in plan_[4].verified_motions])
        sampled_data_distribution_id = self.gmm_._sklearn_gmm.predict(sampled_data_numpy).tolist()


        if plan_[0]:
            # set the transition probability to 1 because the task is successfully completed
            self.task_graph.edges[task_graph_info_]['probability'] = 1.0
        else:
            self.task_graph.edges[task_graph_info_]['probability'] *= 0.5
            # find all similar task in the task graph and decrease their transition probabilities.
            #TODO: implement this