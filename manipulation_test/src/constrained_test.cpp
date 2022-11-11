#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "constrained_test_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);

  spinner.start();
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  // set planner id
  move_group.setPlannerId("CBIRRTConfigDefault");
  //move_group.setPlannerId("CLazyPRMConfigDefault");
  //
  move_group.setPoseTarget(current_pose);
  moveit::planning_interface::MoveGroupInterface::Plan p;
  move_group.plan(p);

  return 0;
}
