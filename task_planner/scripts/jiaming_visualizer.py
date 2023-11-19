import rospy
class BaseVisualizer(object):
    def __init__(self):
        pass

    def prepare_visualizer(self):
        raise NotImplementedError("Please Implement this method")

    def visualize_plan(self, list_of_motion_plan):
        raise NotImplementedError("Please Implement this method")


class MoveitVisualizer(BaseVisualizer):
    def prepare_visualizer(self):
        # this is used to display the planned path in rviz
        self.display_robot_state_publisher = rospy.Publisher(
            "/move_group/result_display_robot_state",
            moveit_msgs.msg.DisplayRobotState,
            queue_size=5,
        )

    def visualize_plan(self, list_of_motion_plan):
        '''
        This function will receive a list of motion plan and visualize it.
        One thing must be considered is that this list contains both motion between intersections and
        intersection. Therefore, the visuliaztion must handle this situation properly.
        '''
        print "visualize the plan"