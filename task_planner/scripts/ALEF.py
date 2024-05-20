import kdtree
import networkx as nx

CONVERAGE = 0
CONNECTIVITY = 1
INTERFACE = 2


class AFS:
    """
    This is a class for the AFS. That is, planning requires to connect local roadmaps from
    similar manifolds, so it can be used as experience for the planner to solve the problem.
    """

    def __init__(self, roadmaps, visible_radius=0.5):
        # need to combine the roadmaps
        # each element in roadmaps is a networkx graph. If two nodes from different roadmaps are close enough, we connect them.
        self.roadmaps = roadmaps
        self.visible_radius = visible_radius
        self.afs = nx.Graph()

        if len(roadmaps) > 0:
            # combine the roadmaps
            # copy the first graph to the afs
            self.afs = self.roadmaps[0].copy()

            for g in self.roadmaps[1:]:
                # compose the graphs if two nodes are close in visible radius
                connected_edges = []
                for node1 in g.nodes:
                    for node2 in self.afs.nodes:
                        if (
                            self.distanceBetweenStates(node1, node2)
                            < self.visible_radius
                        ):
                            connected_edges.append((node1, node2))
                # union the graphs
                self.afs = nx.compose(self.afs, g)
                self.afs.add_edges_from(connected_edges)

    def distanceBetweenStates(self, state1, state2):
        """
        Get the Euclidean distance between two states
        """
        node_diff = [state1[i] - state2[i] for i in range(len(state1))]
        distance = sum([node_diff[i] ** 2 for i in range(len(node_diff))]) ** 0.5

        return distance

    def findClosestNode(self, state):
        """
        Find the closest node to the state in the AFS
        """
        closest_node = None
        min_distance = float("inf")

        for node in self.afs.nodes:
            distance = self.distanceBetweenStates(state, node)
            if distance < min_distance:
                min_distance = distance
                closest_node = node

        return closest_node

    def retrieveExperience(self, start, goal):
        """
        Retrieve experience from the AFS
        """
        # if there is no nodes in the AFS, we return empty list
        if len(self.afs.nodes) == 0:
            return []

        # find the closest node to the start and goal from self.afs
        start_node = self.findClosestNode(start)
        goal_node = self.findClosestNode(goal)

        # if the start and goal are not in the AFS, we return empty list
        if start_node is None or goal_node is None:
            return []

        # if there is no path between the start and goal, we return empty list
        if not nx.has_path(self.afs, start_node, goal_node):
            return []

        # get the path from the AFS
        path = nx.shortest_path(self.afs, start_node, goal_node)

        return path


class SPARSdb:
    """
    This is a class for the SPARSdb roadmap that we implement based on the
    SPARSdb from ompl. We get rid of some functions that are not necessary.
    We only need it to construct sparse map with solution paths for building
    ALEF roadmap.
    """

    def __init__(self, dim):
        self.roadmap = nx.Graph()
        # initialize the kdtree
        self.tree = kdtree.create(dimensions=dim)
        self.sparse_delta_fraction = 0.5

    def setSparseDeltaFraction(self, fraction):
        self.sparse_delta_fraction = fraction

    def getSparseDeltaFraction(self):
        return self.sparse_delta_fraction

    def addPathToRoadmap(self, path):
        """
        Add a path to the roadmap. The guardspacing factor should be a value between 1.9 and 1.1. However,
        we just want something working for now. we set it to be 1.5.
        """
        if len(path) < 2:
            return False

        lastStateID = 0
        connectivityStateIDs = []
        spacingFactor = 1.5

        # add the first state to the roadmap
        self.addStateToRoadmap(path[0])

        for i in range(len(path)):
            distanceFromLastState = self.distanceBetweenStates(
                path[i], path[lastStateID]
            )

            if distanceFromLastState > spacingFactor * self.sparse_delta_fraction:
                # Add the state to the roadmap
                self.addStateToRoadmap(path[i])

                midStateId = (i - lastStateID) // 2 + lastStateID
                connectivityStateIDs.append(midStateId)

                lastStateID = i
            elif i == len(path) - 1:
                self.addStateToRoadmap(path[i])
                midStateID = (i - lastStateID) // 2 + lastStateID
                connectivityStateIDs.append(midStateID)

        for i in range(len(connectivityStateIDs) - 1):
            add_connectibity_node = self.addStateToRoadmap(
                path[connectivityStateIDs[i]]
            )

        return True

    def addStateToRoadmap(self, state):
        """
        Add a state to the roadmap
        """
        stateAdded = False

        graphNeighborhood = self.findGraphNeighbors(state)

        if not self.checkAddCoverage(state, graphNeighborhood):
            if not self.checkAddConnectivity(state, graphNeighborhood):
                if not self.checkAddInterface(state, graphNeighborhood):
                    pass
                else:
                    stateAdded = True
            else:
                stateAdded = True
        else:
            stateAdded = True

        return stateAdded

    def getRoadmap(self):
        """
        Get the roadmap
        """
        return self.roadmap

    ########################################################
    # private functions
    ########################################################

    def checkAddCoverage(self, state, graphNeighborhood):
        """
        Checks to see if the sample needs to be added to ensure coverage of the space.
        """
        if len(graphNeighborhood) > 0:
            return False

        self.addGuard(state, CONVERAGE)

        return True

    def checkAddConnectivity(self, state, graphNeighborhood):
        """
        Checks to see if the sample needs to be added to ensure connectivity.
        """
        # Identify neighbors around the new state that are unconnected and connect them.

        if len(graphNeighborhood) > 1:
            statesInDiffConnectedComponents = []
            for neighbor in graphNeighborhood:
                for neighbor2 in graphNeighborhood:
                    if neighbor != neighbor2:
                        if not nx.has_path(self.roadmap, neighbor, neighbor2):
                            statesInDiffConnectedComponents.append(neighbor)
                            statesInDiffConnectedComponents.append(neighbor2)

            # Were any disconnected states found?
            if len(statesInDiffConnectedComponents) > 0:
                self.addGuard(state, CONNECTIVITY)

                for statesInDiffConnectedComponent in statesInDiffConnectedComponents:
                    # Check if an edge already exists between the states
                    if not self.roadmap.has_edge(state, statesInDiffConnectedComponent):
                        # The components haven't been united by previous links
                        if not nx.has_path(
                            self.roadmap, state, statesInDiffConnectedComponent
                        ):
                            self.connectGuards(state, statesInDiffConnectedComponent)

                return True

        return False

    def distanceBetweenStates(self, state1, state2):
        """
        Get the Euclidean distance between two states
        """
        node_diff = [state1[i] - state2[i] for i in range(len(state1))]
        distance = sum([node_diff[i] ** 2 for i in range(len(node_diff))]) ** 0.5

        return distance

    def checkAddInterface(self, state, graphNeighborhood):
        """
        Checks to see if the current sample reveals the existence of an interface, and if so, tries to bridge it.
        """
        if len(graphNeighborhood) > 1:
            # if the two closest nodes don't share an edge
            if not self.roadmap.has_edge(graphNeighborhood[0], graphNeighborhood[1]):
                # if they are close enough
                if (
                    self.distanceBetweenStates(
                        graphNeighborhood[0], graphNeighborhood[1]
                    )
                    < self.sparse_delta_fraction
                ):
                    self.connectGuards(graphNeighborhood[0], graphNeighborhood[1])
                else:
                    # add the state to the roadmap
                    self.addGuard(state, INTERFACE)
                    self.connectGuards(state, graphNeighborhood[0])
                    self.connectGuards(state, graphNeighborhood[1])
                return True
        return False

    def findGraphNeighbors(self, state):
        """
        Find the neighbors of a state in the graph
        """
        graphNeighborhood = self.tree.search_nn_dist(state, self.sparse_delta_fraction)
        return graphNeighborhood

    def addGuard(self, state, state_type):
        """
        Add a guard to the roadmap
        """
        self.roadmap.add_node(state, state_type=state_type)
        self.tree.add(state)

    def connectGuards(self, state1, state2):
        """
        Connect two guards
        """
        self.roadmap.add_edge(
            state1, state2, weight=self.distanceBetweenStates(state1, state2)
        )
