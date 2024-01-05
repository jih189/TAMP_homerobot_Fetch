import rospy
import numpy as np
import networkx as nx
from foliated_base_class import BaseTaskPlanner, ManifoldDetail, IntersectionDetail, Task
import time
from jiaming_GMM import GMM

from moveit_msgs.srv import ConstructAtlas, ConstructAtlasRequest
from moveit_msgs.msg import ConfigurationWithInfo
from moveit_msgs.srv import ResetAtlas, ResetAtlasRequest

import matplotlib.pyplot as plt

class MTGTaskPlanner(BaseTaskPlanner):
    def __init__(self, planner_name_="MTGTaskPlanner", parameter_dict_={}):
        # Constructor
        super(BaseTaskPlanner, self).__init__() # python 2
        # super().__init__() # python 3
        self.planner_name = planner_name_
        self.parameter_dict = parameter_dict_

    # MTGTaskPlanner
    def reset_task_planner(self):
        self.task_graph = nx.DiGraph()
        self.manifold_info = {} # the detail of each manifold
        self.incomming_manifold_intersections = {} # the incomming intersections of each manifold
        self.outgoing_manifold_intersections = {} # the outgoing intersections of each manifold
        self.new_intersection_id = 0

        # self.reset_manifold_similarity_table()
        self.total_similiarity_table = {}
        
    # MTGTaskPlanner
    def add_manifold(self, manifold_info_, manifold_id_):
        self.manifold_info[manifold_id_] = manifold_info_ # the manifold_info is a dict with keys: foliation, co_parameter.

        self.incomming_manifold_intersections[manifold_id_] = []
        self.outgoing_manifold_intersections[manifold_id_] = []

    # MTGTaskPlanner
    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_detail_):
        intersection_from_1_to_2_id = self.new_intersection_id
        self.new_intersection_id += 1

        # add node for intersection from manifold 1 to manifold 2
        self.task_graph.add_node(intersection_from_1_to_2_id, intersection=intersection_detail_, previous_manifold_id=manifold_id1_, next_manifold_id=manifold_id2_)

        for i in self.incomming_manifold_intersections[manifold_id1_]:
            self.task_graph.add_edge(i, intersection_from_1_to_2_id, weight=0, manifold_id=manifold_id1_)
        for i in self.outgoing_manifold_intersections[manifold_id2_]:
            self.task_graph.add_edge(intersection_from_1_to_2_id, i, weight=0, manifold_id=manifold_id2_)

        self.outgoing_manifold_intersections[manifold_id1_].append(intersection_from_1_to_2_id)
        self.incomming_manifold_intersections[manifold_id2_].append(intersection_from_1_to_2_id)

    # MTGTaskPlanner
    def set_start_and_goal(self,
                            start_manifold_id_,
                            start_intersection_, 
                            goal_manifold_id_,
                            goal_intersection_):


        # if start and goal are set, then remove them from the task graph
        if self.task_graph.has_node('start'):
            self.task_graph.remove_node('start')

        if self.task_graph.has_node('goal'):
            self.task_graph.remove_node('goal')

        configuration_of_start, _ = start_intersection_.get_edge_configurations()
        self.task_graph.add_node(
            'start', 
            intersection = IntersectionDetail(
               start_intersection_,
                configuration_of_start,
                configuration_of_start,
                False
            ),
            previous_manifold_id = 'start',
            next_manifold_id = start_manifold_id_
        )

        configuration_of_goal, _ = goal_intersection_.get_edge_configurations()
        self.task_graph.add_node(
            'goal', 
            intersection = IntersectionDetail(
                goal_intersection_,
                configuration_of_goal,
                configuration_of_goal,
                True
            ),
            previous_manifold_id = goal_manifold_id_,
            next_manifold_id = 'goal'
        )

        for i in self.outgoing_manifold_intersections[start_manifold_id_]:
            self.task_graph.add_edge('start', i, weight=0, manifold_id=start_manifold_id_)

        for i in self.incomming_manifold_intersections[goal_manifold_id_]:
            self.task_graph.add_edge(i, 'goal', weight=0, manifold_id=goal_manifold_id_)

    # MTGTaskPlanner
    def generate_task_sequence(self):
        # check the connectivity of the task graph from start to goal
        if not nx.has_path(self.task_graph, 'start', 'goal'):
            print "no connection between start and goal!"
            return []

        # find the shortest path from start to goal
        shortest_path = nx.shortest_path(self.task_graph, 'start', 'goal', weight='weight')
        task_sequence = []

        # construct the task sequence.
        for node1, node2 in zip(shortest_path[:-1], shortest_path[1:]):

            task = Task(
                        manifold_detail_ =self.manifold_info[self.task_graph.edges[node1, node2]['manifold_id']],
                        start_configuration_=nx.get_node_attributes(self.task_graph, 'intersection')[node1].configuration_in_manifold2,
                        goal_configuration_=nx.get_node_attributes(self.task_graph, 'intersection')[node2].configuration_in_manifold1,
                        next_motion_=nx.get_node_attributes(self.task_graph, 'intersection')[node2].intersection_data
                    )
            
            task.set_task_graph_info((node1, node2))

            task_sequence.append(task)

        return task_sequence

    # MTGTaskPlanner
    def update(self, task_graph_info_, plan_, manifold_constraint_):

        # if current task is faled to solve, then we can increate the weight of the edge which is similar to the current task.
        # the similarity is defined as the product of the similarity of the previous manifold, the next manifold, and the current similarity.
        if not plan_[0]:
            # get the current manifold id, previous manifold id and next manifold id of the task.
            current_manifold_id = self.task_graph.edges[task_graph_info_]['manifold_id']
            previous_manifold_id = self.task_graph.nodes[task_graph_info_[0]]['previous_manifold_id']
            next_manifold_id = self.task_graph.nodes[task_graph_info_[1]]['next_manifold_id']
            
            for (e_start_node, e_goal_node, e_current_manifold_id) in self.task_graph.edges.data('manifold_id'):
                # find all the edges having the same foliation with the current task.
                if current_manifold_id[0] == e_current_manifold_id[0]:
                    e_previous_manifold_id = self.task_graph.nodes[e_start_node]['previous_manifold_id']
                    e_next_manifold_id = self.task_graph.nodes[e_goal_node]['next_manifold_id']

                    previous_similarity_score = 0
                    next_similarity_score = 0

                    if (previous_manifold_id == 'start' or e_previous_manifold_id == 'start'):
                        if previous_manifold_id == e_previous_manifold_id:
                            previous_similarity_score = 1.0
                        else:
                            # previous similarity score is 0, so we can skip this edge.
                            continue
                    else:
                        previous_similarity_score = self.total_similiarity_table[previous_manifold_id[0]][e_previous_manifold_id[1], previous_manifold_id[1]]

                    if (next_manifold_id == 'goal' or e_next_manifold_id == 'goal'):
                        if next_manifold_id == e_next_manifold_id:
                            next_similarity_score = 1.0
                        else:
                            # next similarity score is 0, so we can skip this edge.
                            continue
                    else:
                        next_similarity_score = self.total_similiarity_table[next_manifold_id[0]][e_next_manifold_id[1], next_manifold_id[1]]

                    current_similarity_score = self.total_similiarity_table[current_manifold_id[0]][e_current_manifold_id[1], current_manifold_id[1]]

                    total_similarity_score = current_similarity_score * previous_similarity_score * next_similarity_score

                    self.task_graph.edges[(e_start_node, e_goal_node)]['weight'] += 1.0 * total_similarity_score

class MTGTaskPlannerWithGMM(BaseTaskPlanner):
    def __init__(self, gmm, planner_name_="MTGTaskPlannerWithGMM", parameter_dict_={}):
        # Constructor
        super(BaseTaskPlanner, self).__init__()
        # super().__init__() # python 3

        self.gmm_ = gmm

        self.planner_name = planner_name_

        self.parameter_dict = parameter_dict_

    # MTGTaskPlannerWithGMM
    def reset_task_planner(self):

        self.task_graph = nx.DiGraph()
        self.manifold_info = {} # the constraints of each manifold

        # self.reset_manifold_similarity_table()
        self.total_similiarity_table = {}

    # MTGTaskPlannerWithGMM
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

            # need to add the inverse edge
            self.task_graph.add_edge(
                (manifold_id_[0], manifold_id_[1], edge[1]), 
                (manifold_id_[0], manifold_id_[1], edge[0]),
                has_intersection=False,
                intersection=None
            )

    # MTGTaskPlannerWithGMM
    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_detail_):

        # connect two distribution of this intersection_detail_ between two different manifolds(manifold1 and manifold2) if they have the same ditribution id in GMM.
        # first, find the related distribution that the intersection's ends are in in different manifolds.

        distribution_id_in_manifold1, distribution_id_in_manifold2 = self.gmm_.get_distribution_indexs([intersection_detail_.configuration_in_manifold1, intersection_detail_.configuration_in_manifold2])

        # intersection_from_1_to_2_id = self.add_intersection_for_task_solution_graph(manifold_id1_, manifold_id2_)

        self.task_graph.add_edge(
            (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1), 
            (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2),
            has_intersection=True, 
            intersection=intersection_detail_
        )

    # MTGTaskPlannerWithGMM
    def set_start_and_goal(self,
                            start_manifold_id_,
                            start_intersection_, 
                            goal_manifold_id_,
                            goal_intersection_):

        # self.set_start_and_goal_for_task_solution_graph(start_manifold_id_, goal_manifold_id_)

        # if start and goal are set, then remove them from the task graph
        if self.task_graph.has_node('start'):
            self.task_graph.remove_node('start')

        if self.task_graph.has_node('goal'):
            self.task_graph.remove_node('goal')

        # include start and goal configurations in the task graph
        self.task_graph.add_node('start', weight = 0.0)
        self.task_graph.add_node('goal', weight = 0.0)

        configuration_of_start, _ = start_intersection_.get_edge_configurations()
        self.task_graph.add_edge(
            'start', 
            (
                start_manifold_id_[0], 
                start_manifold_id_[1], 
                self.gmm_.get_distribution_index(np.array(configuration_of_start))
            ), 
            has_intersection=False, 
            intersection=None
        )

        configuration_of_goal, _ = goal_intersection_.get_edge_configurations()
        self.task_graph.add_edge(
            (
                goal_manifold_id_[0], 
                goal_manifold_id_[1], 
                self.gmm_.get_distribution_index(np.array(configuration_of_goal))
            ), 
            'goal', 
            has_intersection=True, 
            intersection=IntersectionDetail(
                goal_intersection_,
                configuration_of_goal,
                configuration_of_goal,
                True
            )
        )

        self.current_start_configuration = configuration_of_start

    # MTGTaskPlannerWithGMM
    def generate_task_sequence(self):
        # print the number of nodes can achieve the goal
        # print "number of nodes can achieve the goal: ", len([node for node in self.task_graph.nodes if nx.has_path(self.task_graph, node, 'goal')])

        # check the connectivity of the task graph from start to goal
        if not nx.has_path(self.task_graph, 'start', 'goal'):
            print "no connection between start and goal!"
            return []

        # find the shortest path from start to goal
        shortest_path = nx.shortest_path(self.task_graph, 'start', 'goal', weight='weight')

        task_sequence = []

        task_start_configuration = self.current_start_configuration
        task_node_experience = [] # each element is a tuple (node_id, distribution, list of related task nodes)
        # in MTGGMM Task planner, we do not use the list of related task nodes here.

        # construct the task sequence.
        for node1, node2 in zip(shortest_path[:-1], shortest_path[1:]):
            current_edge = self.task_graph.get_edge_data(node1, node2)

            if current_edge['has_intersection']:
                # current edge is a transition from one manifold to another manifold
                task = Task(
                    manifold_detail_=self.manifold_info[node1[:2]],
                    start_configuration_=task_start_configuration, # start configuration of the task
                    goal_configuration_=current_edge['intersection'].configuration_in_manifold1, # target configuration of the task
                    next_motion_=current_edge['intersection'].intersection_data # the motion after the task.
                )

                task.related_experience = list(task_node_experience)

                # we use the intersection id as task graph information here
                # the task graph information contains the manifold id of the current task.
                task.set_task_graph_info(node1[:2])

                task_sequence.append(task)

                # ready for the next task.
                if node2 != 'goal': # if the edge is to goal, then no need to prepare for the next task
                    task_node_experience = [(node2, self.gmm_.distributions[node2[2]], [])]
                    # consider the last state of the intersection motion as the start state of next task.
                    task_start_configuration = current_edge['intersection'].configuration_in_manifold2
            else:
                # edge in the same manifold except start and goal transition
                task_node_experience.append((node2, self.gmm_.distributions[node2[2]], []))

        return task_sequence

    # MTGTaskPlannerWithGMM
    def update(self, task_graph_info_, plan_, manifold_constraint_):
        """
        After planning a motion task in a foliated manifold M(f', c'), we receive a set of configuration with its status.
        Where f', c' are the foliation id and co-parameter id define the current task's manifold.
        The sampled_data_distribution_tag_table is a table with shape (number of distributions in GMM, 4).
        Each row is a distribution in GMM, and each column is a tag of sampled data.
        The value in the table is the number of sampled data with the same distribution id and tag.

        Then, we need to update the weight of all nodes in the task graph having the same foliation with the foliated manifold M(f', c').
        For each node (f, c, d) where f is the foliation id, c is the co-parameter id, and d is the distribution id, we update the weight of the node by:
        current_similarity_score is the similarty between c and c' in the foliation f.
        arm_env_collision_score = sampled_data_distribution_tag_table[d][1] * 1.0
        path_constraint_violation_score = current_similarity_score * sampled_data_distribution_tag_table[d][2] * 1.0
        obj_env_collision_score = current_similarity_score * sampled_data_distribution_tag_table[d][3] * 1.0
        weight = weight + arm_env_collision_score + path_constraint_violation_score + obj_env_collision_score
        """
        # use the sample data to update the task graph.
        # sampled_state_tag hint
        # 0: collision free
        # 1: arm-env collision or out of joint limit
        # 2: path constraint violation
        # 3: infeasble state, you should ignore this
        # 4: obj-env collision
        # 5: valid configuration before project
        # 6: arm-env collision or out of joint limit before project
        # 7: path constraint violation before project
        # 8: infeasble state, you should ignore this before project
        # 9: obj-env collision before project


        # if sampled data is empty, then skip it.
        if len(plan_[4].verified_motions) == 0:
            print "sampled data is empty."
            return

        sampled_data_numpy = np.array([sampled_data.sampled_state for sampled_data in plan_[4].verified_motions])
        
        # if sampled_data_numpy is empty, then skip it.
        sampled_data_distribution_id = self.gmm_._sklearn_gmm.predict(sampled_data_numpy).tolist()

        # the task graph info here is the manifold id(foliatino id and co-parameter id) of the current task.
        current_manifold_id = task_graph_info_

        # initialize a table with number of distributions in GMM times 4.
        # each row is a distribution in GMM, and each column is a tag of sampled data.
        # the value in the table is the number of sampled data with the same distribution id and tag.
        # tag in column 0: collision free
        # tag in column 1: arm-env collision or out of joint limit
        # tag in column 2: path constraint violation
        # tag in column 3: obj-env collision
        sampled_data_distribution_tag_table = np.zeros((len(self.gmm_.distributions), 4))

        # count the number of sampled data with the same distribution id and tag.
        for i in range(len(sampled_data_distribution_id)):
            sampled_data_gmm_id = sampled_data_distribution_id[i]
            sampled_data_tag = plan_[4].verified_motions[i].sampled_state_tag

            if sampled_data_tag == 0 or sampled_data_tag == 5:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][0] += 1
            elif sampled_data_tag == 1 or sampled_data_tag == 6:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][1] += 1
            elif sampled_data_tag == 2 or sampled_data_tag == 7:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][2] += 1
            elif sampled_data_tag == 4 or sampled_data_tag == 9:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][3] += 1

        # only update the weight of nodes in the same manifold with the current task.
        for n in self.task_graph.nodes():
            if n == 'start' or n == 'goal':
                continue
            
            # if not in the same foliation, then continue
            if n[0] != current_manifold_id[0]:
                continue

            node_foliation_id = n[0]
            node_co_parameter_id = n[1]
            node_gmm_id = n[2]

            current_similarity_score = self.total_similiarity_table[node_foliation_id][node_co_parameter_id, current_manifold_id[1]]


            arm_env_collision_score = sampled_data_distribution_tag_table[node_gmm_id][1] * 1.0
            path_constraint_violation_score = current_similarity_score * sampled_data_distribution_tag_table[node_gmm_id][2] * 1.0
            obj_env_collision_score = current_similarity_score * sampled_data_distribution_tag_table[node_gmm_id][3] * 1.0

            self.task_graph.nodes[n]['weight'] += arm_env_collision_score + path_constraint_violation_score + obj_env_collision_score

        for u, v in self.task_graph.edges():
            self.task_graph.edges[u, v]['weight'] = self.task_graph.nodes[v]['weight'] + self.task_graph.nodes[u]['weight']

class MTGTaskPlannerWithAtlas(BaseTaskPlanner):
    def __init__(self, gmm, default_robot_state, planner_name_="MTGTaskPlannerWithAtlas", parameter_dict_={}):
        # Constructor
        super(BaseTaskPlanner, self).__init__()
        # super().__init__() # python 3

        self.gmm_ = gmm
        self.default_robot_state_ = default_robot_state

        self.planner_name = planner_name_

        self.parameter_dict = parameter_dict_

        self.atlas_service = rospy.ServiceProxy('/construct_atlas', ConstructAtlas)

        self.reset_atlas_service = rospy.ServiceProxy('/reset_atlas', ResetAtlas)

    # MTGTaskPlannerWithAtlas
    def reset_task_planner(self):

        self.task_graph = nx.DiGraph()
        self.manifold_info = {} # the constraints of each manifold
        
        self.foliation_with_co_parameter_id = {} # the co-parameter id of each foliation

        # self.reset_manifold_similarity_table()
        self.total_similiarity_table = {}

    # MTGTaskPlannerWithAtlas
    def add_manifold(self, manifold_info_, manifold_id_):

        self.manifold_info[manifold_id_] = manifold_info_

        if manifold_id_[0] not in self.foliation_with_co_parameter_id:
            self.foliation_with_co_parameter_id[manifold_id_[0]] = [manifold_id_[1]]
        else:
            self.foliation_with_co_parameter_id[manifold_id_[0]].append(manifold_id_[1])

        # construct a set of nodes represented by a tuple (foliation id, manifold id, GMM id)
        for i in range(len(self.gmm_.distributions)):
            self.task_graph.add_node((manifold_id_[0], manifold_id_[1], i), weight = 0.0, has_atlas=False, valid_configuration_before_project = 0, invalid_configuration_before_project = 0)

        for edge in self.gmm_.edge_of_distribution:
            self.task_graph.add_edge(
                (manifold_id_[0], manifold_id_[1], edge[0]), 
                (manifold_id_[0], manifold_id_[1], edge[1]),
                has_intersection=False,
                intersection=None
            )

            # need to add the inverse edge
            self.task_graph.add_edge(
                (manifold_id_[0], manifold_id_[1], edge[1]), 
                (manifold_id_[0], manifold_id_[1], edge[0]),
                has_intersection=False,
                intersection=None
            )

    # MTGTaskPlannerWithAtlas
    def add_intersection(self, manifold_id1_, manifold_id2_, intersection_detail_):

        # connect two distribution of this intersection_detail_ between two different manifolds(manifold1 and manifold2) if they have the same ditribution id in GMM.
        # first, find the related distribution that the intersection's ends are in in different manifolds.

        distribution_id_in_manifold1, distribution_id_in_manifold2 = self.gmm_.get_distribution_indexs([intersection_detail_.configuration_in_manifold1, intersection_detail_.configuration_in_manifold2])

        # intersection_from_1_to_2_id = self.add_intersection_for_task_solution_graph(manifold_id1_, manifold_id2_)

        self.task_graph.add_edge(
            (manifold_id1_[0], manifold_id1_[1], distribution_id_in_manifold1), 
            (manifold_id2_[0], manifold_id2_[1], distribution_id_in_manifold2),
            has_intersection=True, 
            intersection=intersection_detail_
        )

    # MTGTaskPlannerWithAtlas
    def set_start_and_goal(self,
                            start_manifold_id_,
                            start_intersection_, 
                            goal_manifold_id_,
                            goal_intersection_):

        # reset the atlas
        self.reset_atlas_service.call(ResetAtlasRequest())
        
        # if start and goal are set, then remove them from the task graph
        if self.task_graph.has_node('start'):
            self.task_graph.remove_node('start')

        if self.task_graph.has_node('goal'):
            self.task_graph.remove_node('goal')

        # include start and goal configurations in the task graph
        self.task_graph.add_node('start', weight = 0.0)
        self.task_graph.add_node('goal', weight = 0.0)

        configuration_of_start, _ = start_intersection_.get_edge_configurations()
        self.task_graph.add_edge(
            'start', 
            (
                start_manifold_id_[0], 
                start_manifold_id_[1], 
                self.gmm_.get_distribution_index(np.array(configuration_of_start))
            ), 
            has_intersection=False, 
            intersection=None
        )

        configuration_of_goal, _ = goal_intersection_.get_edge_configurations()
        self.task_graph.add_edge(
            (
                goal_manifold_id_[0], 
                goal_manifold_id_[1], 
                self.gmm_.get_distribution_index(np.array(configuration_of_goal))
            ), 
            'goal', 
            has_intersection=True, 
            intersection=IntersectionDetail(
                goal_intersection_,
                configuration_of_goal,
                configuration_of_goal,
                True
            )
        )

        self.current_start_configuration = configuration_of_start

    # MTGTaskPlannerWithAtlas
    def generate_task_sequence(self):
        # print the number of nodes can achieve the goal
        # print "number of nodes can achieve the goal: ", len([node for node in self.task_graph.nodes if nx.has_path(self.task_graph, node, 'goal')])

        # check the connectivity of the task graph from start to goal
        if not nx.has_path(self.task_graph, 'start', 'goal'):
            print "no connection between start and goal!"
            return []

        # find the shortest path from start to goal
        shortest_path = nx.shortest_path(self.task_graph, 'start', 'goal', weight='weight')

        task_sequence = []

        task_start_configuration = self.current_start_configuration
        task_node_experience = [] # each element contains a tuple (node, distribution, list of related task nodes)

        # construct the task sequence.
        for node1, node2 in zip(shortest_path[:-1], shortest_path[1:]):
            current_edge = self.task_graph.get_edge_data(node1, node2)

            if current_edge['has_intersection']:
                # current edge is a transition from one manifold to another manifold
                task = Task(
                    manifold_detail_=self.manifold_info[node1[:2]],
                    start_configuration_=task_start_configuration, # start configuration of the task
                    goal_configuration_=current_edge['intersection'].configuration_in_manifold1, # target configuration of the task
                    next_motion_=current_edge['intersection'].intersection_data # the motion after the task.
                )

                task.related_experience = list(task_node_experience)

                # we use the intersection id as task graph information here
                # the task graph information contains the manifold id  of the current task.
                task.set_task_graph_info(node1[:2])

                task_sequence.append(task)

                # ready for the next task.
                if node2 != 'goal': # if the edge is to goal, then no need to prepare for the next task
                    task_node_experience = [(node2, self.gmm_.distributions[node2[2]], self.get_related_task_nodes(node2))]
                    # consider the last state of the intersection motion as the start state of next task.
                    task_start_configuration = current_edge['intersection'].configuration_in_manifold2
            else:
                # edge in the same manifold except start and goal transition
                task_node_experience.append((node2, self.gmm_.distributions[node2[2]], self.get_related_task_nodes(node2)))

        return task_sequence

    # MTGTaskPlannerWithAtlas
    def get_related_task_nodes(self, current_node):
        '''
        Return a list of co_parameter with beta value and similarity score.
        '''
        result = []
        for co_parameter_index in self.foliation_with_co_parameter_id[current_node[0]]:
            if self.task_graph.nodes[(current_node[0], co_parameter_index, current_node[2])]['has_atlas']:
                num_of_configuration_before_project = self.task_graph.nodes[(current_node[0], co_parameter_index, current_node[2])]['valid_configuration_before_project'] + self.task_graph.nodes[(current_node[0], co_parameter_index, current_node[2])]['invalid_configuration_before_project']
                num_of_invalid_configuration_before_project = self.task_graph.nodes[(current_node[0], co_parameter_index, current_node[2])]['invalid_configuration_before_project']
                similarity_score = self.total_similiarity_table[current_node[0]][co_parameter_index, current_node[1]]
                if num_of_configuration_before_project == 0:
                    result.append((
                        co_parameter_index, 
                        1.0,
                        similarity_score # if no configuration before project, then we assume the volume of this region is very thin.
                    )) # related task nodes contains all the nodes in the same foliation with the same distribution id.
                else:
                    result.append((
                        co_parameter_index, 
                        (num_of_invalid_configuration_before_project * 1.0) / num_of_configuration_before_project,
                        similarity_score
                    )) # related task nodes contains all the nodes in the same foliation with the same distribution id.
        return result


    # MTGTaskPlannerWithAtlas
    def update(self, task_graph_info_, plan_, manifold_constraint_):
        """
        After planning a motion task in a foliated manifold M(f', c'), we receive a set of configuration with its status.
        Where f', c' are the foliation id and co-parameter id define the current task's manifold.
        The sampled_data_distribution_tag_table is a table with shape (number of distributions in GMM, 4).
        Each row is a distribution in GMM, and each column is a tag of sampled data.
        The value in the table is the number of sampled data with the same distribution id and tag.

        Then, we need to update the weight of all nodes in the task graph having the same foliation with the foliated manifold M(f', c').
        For each node (f, c, d) where f is the foliation id, c is the co-parameter id, and d is the distribution id, we update the weight of the node by:
        current_similarity_score is the similarty between c and c' in the foliation f.
        arm_env_collision_score = sampled_data_distribution_tag_table[d][1] * 1.0
        path_constraint_violation_score = current_similarity_score * sampled_data_distribution_tag_table[d][2] * 1.0
        obj_env_collision_score = current_similarity_score * sampled_data_distribution_tag_table[d][3] * 1.0
        weight = weight + arm_env_collision_score + path_constraint_violation_score + obj_env_collision_score
        """
        # use the sample data to update the task graph.
        # sampled_state_tag hint
        # 0: collision free
        # 1: arm-env collision or out of joint limit
        # 2: path constraint violation
        # 3: infeasble state, you should ignore this
        # 4: obj-env collision
        # ---
        # 5: valid configuration before project
        # 6: arm-env collision or out of joint limit before project
        # 7: path constraint violation before project
        # 8: infeasble state, you should ignore this before project
        # 9: obj-env collision before project

        # if sampled data is empty, then skip it.
        if len(plan_[4].verified_motions) == 0:
            print "sampled data is empty."
            return

        sampled_data_numpy = np.array([sampled_data.sampled_state for sampled_data in plan_[4].verified_motions])

        if np.isnan(sampled_data_numpy).any():
            print "sampled data contains nan."
            print sampled_data_numpy
            return
        
        # if sampled_data_numpy is empty, then skip it.
        sampled_data_distribution_id = self.gmm_._sklearn_gmm.predict(sampled_data_numpy).tolist()

        # the task graph info here is the manifold id(foliatino id and co-parameter id) of the current task.
        current_manifold_id = task_graph_info_

        # initialize a table with number of distributions in GMM times 4.
        # each row is a distribution in GMM, and each column is a tag of sampled data.
        # the value in the table is the number of sampled data with the same distribution id and tag.
        # tag in column 0: collision free
        # tag in column 1: arm-env collision or out of joint limit
        # tag in column 2: path constraint violation
        # tag in column 3: obj-env collision
        # tag in column 4: valid configuration before project
        # tag in column 5: invalid configuration before project
        sampled_data_distribution_tag_table = np.zeros((len(self.gmm_.distributions), 6))

        construct_atlas_request = ConstructAtlasRequest()
        construct_atlas_request.group_name = "arm"
        construct_atlas_request.foliation_id = task_graph_info_[0]
        construct_atlas_request.co_parameter_id = task_graph_info_[1]
        construct_atlas_request.list_of_configuration_with_info = []
        construct_atlas_request.default_state = self.default_robot_state_
        construct_atlas_request.constraints = manifold_constraint_

        # count the number of sampled data with the same distribution id and tag.
        for i in range(len(sampled_data_distribution_id)):
            sampled_data_gmm_id = sampled_data_distribution_id[i]
            sampled_data_tag = plan_[4].verified_motions[i].sampled_state_tag

            if sampled_data_tag == 0 or sampled_data_tag == 5:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][0] += 1
                configuration_with_info = ConfigurationWithInfo()
                configuration_with_info.joint_configuration = plan_[4].verified_motions[i].sampled_state
                configuration_with_info.distribution_id = sampled_data_gmm_id
                construct_atlas_request.list_of_configuration_with_info.append(configuration_with_info)
            elif sampled_data_tag == 1 or sampled_data_tag == 6:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][1] += 1
            elif sampled_data_tag == 2 or sampled_data_tag == 7:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][2] += 1
            elif sampled_data_tag == 4 or sampled_data_tag == 9:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][3] += 1
            
            if sampled_data_tag == 5:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][4] += 1
            elif sampled_data_tag > 5:
                sampled_data_distribution_tag_table[sampled_data_gmm_id][5] += 1
        
        if len(construct_atlas_request.list_of_configuration_with_info) != 0:
            self.atlas_service.call(construct_atlas_request)

        # only update the weight of nodes in the same manifold with the current task.
        for n in self.task_graph.nodes():
            if n == 'start' or n == 'goal':
                continue
            
            # if not in the same foliation, then continue
            if n[0] != current_manifold_id[0]:
                continue

            node_foliation_id = n[0]
            node_co_parameter_id = n[1]
            node_gmm_id = n[2]

            current_similarity_score = self.total_similiarity_table[node_foliation_id][node_co_parameter_id, current_manifold_id[1]]

            arm_env_collision_score = sampled_data_distribution_tag_table[node_gmm_id][1] * 1.0
            path_constraint_violation_score = current_similarity_score * sampled_data_distribution_tag_table[node_gmm_id][2] * 1.0
            obj_env_collision_score = current_similarity_score * sampled_data_distribution_tag_table[node_gmm_id][3] * 1.0

            self.task_graph.nodes[n]['weight'] += arm_env_collision_score + path_constraint_violation_score + obj_env_collision_score

        for u, v in self.task_graph.edges():
            self.task_graph.edges[u, v]['weight'] = self.task_graph.nodes[v]['weight'] + self.task_graph.nodes[u]['weight']
        
        # update the valid configuration before project and invalid configuration before project
        for distribution_index in range(len(self.gmm_.distributions)):
            self.task_graph.nodes[(current_manifold_id[0], current_manifold_id[1], distribution_index)]['valid_configuration_before_project'] += sampled_data_distribution_tag_table[distribution_index][4]
            self.task_graph.nodes[(current_manifold_id[0], current_manifold_id[1], distribution_index)]['invalid_configuration_before_project'] += sampled_data_distribution_tag_table[distribution_index][5]
            # if there are some projected valid configuration, then there must be an atlas.
            if sampled_data_distribution_tag_table[distribution_index][0] > 0 or sampled_data_distribution_tag_table[distribution_index][4] > 0:
                self.task_graph.nodes[(current_manifold_id[0], current_manifold_id[1], distribution_index)]['has_atlas'] = True