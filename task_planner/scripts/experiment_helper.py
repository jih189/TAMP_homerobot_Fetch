"""
experiment helper
------------------
Here are the helper classes to save and load experiments. Thus, later the task planner
can use this code to load and construct the task graph.
"""
class Manifold:
    """
    Manifold is the class describe all information of a constraint manifold in foliations.
    """

    def __init__(self, foliation_id_, manifold_id_, object_name_, object_mesh_, in_hand_pose_, orientation_constraint_, position_constraint_):
        """
        Initialize the class instance.
        """
        self.foliation_id = foliation_id_
        self.manifold_id = manifold_id_
        self.object_name = object_name_
        self.object_mesh = object_mesh_
        self.in_hand_pose = in_hand_pose_
        self.orientation_constraint = orientation_constraint_
        self.position_constraint = position_constraint_

class Intersection:
    """
    Intersection describes the motion across two different manifolds.
    """
    def __init__(self, foliation_id_1_, manifold_id_1_, foliation_id_2_, manifold_id_2_, has_object_in_hand_, trajectory_motion_, in_hand_pose_, object_mesh_, object_name_):
        self.foliation_id_1 = foliation_id_1_
        self.manifold_id_1 = manifold_id_1_
        self.foliation_id_2 = foliation_id_2_
        self.manifold_id_2 = manifold_id_2_
        self.has_object_in_hand = has_object_in_hand_
        self.trajectory_motion =  trajectory_motion_
        self.in_hand_pose = in_hand_pose_
        self.object_mesh = object_mesh_
        self.object_name = object_name_

# Usage example
if __name__ == "__main__":
    # Instantiate objects and demonstrate the usage of the class
