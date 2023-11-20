import rospy
from foliated_base_class import BaseIntersection, BaseTaskMotion, BaseVisualizer

class ManipulationTaskMotion(BaseTaskMotion):
    def get(self):
        #TODO implement this function
        return self.motion_plan


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
        print "number of motion plan: ", len(list_of_motion_plan)
        print "visualize the plan"
        print "press ctrl+c to exit"

        for motion_plan in list_of_motion_plan:
            # if motion plan is base intersection
            if isinstance(motion_plan, BaseIntersection):
                print "type of motion plan: ", type(motion_plan)
            else:
                print "type of motion plan: ", type(motion_plan[1])
        # while not rospy.is_shutdown():

