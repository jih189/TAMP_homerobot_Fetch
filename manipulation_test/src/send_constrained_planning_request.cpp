#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <visualization_msgs/Marker.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "manipulation_test/experience_graph.hpp"


void geoPoseToTransformTF(const geometry_msgs::Pose &p, tf::Transform &t){
  t.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z));
  t.setRotation(tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
}

void transformTFToGeoPose(const tf::Transform &t, geometry_msgs::Pose &p){
  p.orientation.x = t.getRotation().x();
  p.orientation.y = t.getRotation().y();
  p.orientation.z = t.getRotation().z();
  p.orientation.w = t.getRotation().w();
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
}

void productBetweenGeoPose(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, geometry_msgs::Pose &result){
  tf::Transform p1t;
  tf::Transform p2t;
  geoPoseToTransformTF(p1, p1t);
  geoPoseToTransformTF(p2, p2t);
  transformTFToGeoPose(p1t*p2t, result);
}

void transformTFToKDL(const tf::Transform &t, KDL::Frame &k){
  for(uint i = 0; i < 3; ++i)
    k.p[i] = t.getOrigin()[i];
  for(uint i = 0; i < 9; ++i)
    k.M.data[i] = t.getBasis()[i/3][i%3]; 
}

int main(int argc, char** argv)
{
  
  // predefine in-hand object poses
  std::vector<tf::Transform> in_hand_poses;

  float positions[5][3] = {
	  		{0.202,0,-0.025},
	  		{0.202,0,-0.025},
	  		{0.202,0,-0.025},
	  		{0.202,0,-0.025},
	  		{0.202,0,-0.025}
  			};
  float rotations[5][4] = {
	  		{-0.3,0.067,0.947,-0.084},
	  		{-0.303,-0.052,0.907,0.284},
	  		{-0.245,-0.186,0.674,0.671},
	  		{-0.177,-0.251,0.44,0.843},
	  		{-0.041,-0.305,0.001,0.951}
  			};


  for(int i = 0; i < 5; i++){
    tf::Transform one_in_hand_pose;
    one_in_hand_pose.setOrigin(tf::Vector3(positions[i][0], positions[i][1], positions[i][2]));
    one_in_hand_pose.setRotation(tf::Quaternion(rotations[i][0],rotations[i][1],rotations[i][2],rotations[i][3]));
    in_hand_poses.push_back(one_in_hand_pose);
  }

  ros::init(argc, argv, "send_constrained_planning_request");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string OBJECT_NAME = "can";
  static const std::string OBJECT_MESH_FILE = "package://objects_description/can/can.stl";
  static const std::string PLANNING_GROUP = "arm";
  static const std::string END_EFFECTOR_PLANNING_GROUP = "gripper";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface end_effector_move_group(END_EFFECTOR_PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  // get the robot kinematic model
  robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

  // extract the pose of the object 
  tf::TransformListener listener;

  //Eigen::Affine3d object_in_world_frame;
  tf::StampedTransform object_transform;
  tf::StampedTransform shoulder_transform;
  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/" + OBJECT_NAME, "/base_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("/base_link","/" + OBJECT_NAME, ros::Time(0), object_transform);
    listener.waitForTransform("/base_link", "/shoulder_pan_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("/base_link","/shoulder_pan_link", ros::Time(0), shoulder_transform);
  }
  catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // add the object into the planning scene
  moveit_msgs::CollisionObject collision_object_;
  collision_object_.header.frame_id = move_group.getPlanningFrame();
  collision_object_.id = OBJECT_NAME; 
  std::string object_dir = OBJECT_MESH_FILE;
  const Eigen::Vector3d scaling(0.001,0.001,0.001);
  shapes::Mesh *m = shapes::createMeshFromResource(object_dir, scaling);
  shape_msgs::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m, object_mesh_msg);
  object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);

  geometry_msgs::Pose object_pose;
  transformTFToGeoPose(object_transform, object_pose);

  collision_object_.meshes.push_back(object_mesh);
  collision_object_.pose = object_pose;
  
  // add the table into the planning scene
  moveit_msgs::CollisionObject collision_object_table;
  collision_object_table.header.frame_id = move_group.getPlanningFrame();

  collision_object_table.id = "table";

  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[0] = 1.2;
  table_primitive.dimensions[1] = 1.2;
  table_primitive.dimensions[2] = 0.2;

  geometry_msgs::Pose table_pose;
  table_pose.position.x = 0.91;
  table_pose.position.y = 0;
  table_pose.position.z = 0.445;

  table_pose.orientation.x = 0;
  table_pose.orientation.y = 0;
  table_pose.orientation.z = 0;
  table_pose.orientation.w = 1;


  collision_object_table.primitives.push_back(table_primitive);
  collision_object_table.pose = table_pose;

  collision_object_table.operation = collision_object_table.ADD;
  planning_scene_interface.applyCollisionObject(collision_object_table);

  // initialize the visualizer for grasps
  moveit_visual_tools::MoveItVisualToolsPtr grasp_visuals = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link", "/move_group/display_contacts");
  rviz_visual_tools::RvizVisualToolsPtr targetObject_visuals = std::make_shared<rviz_visual_tools::RvizVisualTools>("base_link", "/target_object_pose");
  targetObject_visuals->deleteAllMarkers();
  moveit_visual_tools::MoveItVisualToolsPtr arm_visuals = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link");

  // initialize the trac ik solver
  TRAC_IK::TRAC_IK tracik_solver(std::string("torso_lift_link"), std::string("wrist_roll_link"));
  KDL::Chain chain;
  KDL::JntArray ll, ul;

  bool valid_trac_ik = tracik_solver.getKDLChain(chain);
  if(!valid_trac_ik){
    ROS_ERROR("There was no valid KDL chain found");
    return 0;
  }

  valid_trac_ik = tracik_solver.getKDLLimits(ll, ul);
  if(!valid_trac_ik){
    ROS_ERROR("There were no valid KDL joint limits found");
    return 0;
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::ChainIkSolverVel_pinv vik_solver(chain);
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver);//, 1, 1e-5);

  KDL::JntArray nominal(chain.getNrOfJoints());

  for(uint j = 0; j < nominal.data.size(); j++){
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  bool foundSolution = false;
  moveit::core::RobotState current_state = *(move_group.getCurrentState());
  geometry_msgs::Pose current_in_hand_pose;
  geometry_msgs::Pose current_in_hand_pose_inv;

  // initialize the whole trajectory
  robot_trajectory::RobotTrajectory total_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);

  // analyze where to grasp the object
  for(int i = 0; i < in_hand_poses.size(); i++){
    total_trajectory.clear();

    transformTFToGeoPose(in_hand_poses[i], current_in_hand_pose);
    transformTFToGeoPose(in_hand_poses[i].inverse(), current_in_hand_pose_inv);
    // for each possible grasp poses, there are three motion we need to plan
    // 1. pre-grasp planning
    // 2. approach planning
    // 3. lift planning

    // reset the start robot state
    current_state = *(move_group.getCurrentState());
    current_state.setJointPositions(std::string("l_gripper_finger_joint"), std::vector<double>{0.04});
    current_state.setJointPositions(std::string("r_gripper_finger_joint"), std::vector<double>{0.04});

    // calculate the grasp pose in the world frame
    tf::Transform grasp_pose_in_world_frame = object_transform * in_hand_poses[i].inverse(); 
    geometry_msgs::Pose grasp_pose;
    transformTFToGeoPose(grasp_pose_in_world_frame, grasp_pose);

    // calculate the pre-grasp pose in the world frame
    tf::Transform pre_grasp_pose_in_world_frame = grasp_pose_in_world_frame * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.08, 0, 0));
    geometry_msgs::Pose pre_grasp_pose;
    transformTFToGeoPose(pre_grasp_pose_in_world_frame, pre_grasp_pose);

    // calculate the lift up pose in the world frame
    tf::Transform lift_up_pose_in_world_frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0.1)) * grasp_pose_in_world_frame;
    geometry_msgs::Pose lift_up_pose;
    transformTFToGeoPose(lift_up_pose_in_world_frame, lift_up_pose);
    
    grasp_visuals->publishEEMarkers(pre_grasp_pose, grasp_visuals->getSharedRobotState()->getJointModelGroup(END_EFFECTOR_PLANNING_GROUP), std::vector<double>{0.04,0.04}, rviz_visual_tools::RED, "pre-grasp_pose_" + std::to_string(i));
    
    // check whether the pre-grasp pose is feasible
    KDL::Frame end_effector_pose;
    KDL::JntArray result = nominal;
    transformTFToKDL(shoulder_transform.inverse() * pre_grasp_pose_in_world_frame, end_effector_pose);
    if(kdl_solver.CartToJnt(result, end_effector_pose, result) < 0)
      continue;

    // add the object into the the planning scene
    std::vector<std::string> current_object_ids = planning_scene_interface.getKnownObjectNames();
    if(std::find(current_object_ids.begin(), current_object_ids.end(), OBJECT_NAME) == current_object_ids.end()){
      collision_object_.operation = collision_object_.ADD;
      planning_scene_interface.applyCollisionObject(collision_object_);
    }
    current_object_ids.clear();

    // check whether the pre-grasping motion is possible
    move_group.setStartState(current_state);
    move_group.setPoseTarget(pre_grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    if(move_group.plan(grasp_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS){
      move_group.clearPoseTargets();
      continue;
    }

    move_group.clearPoseTargets();
    // generate the grasp_trajectory object and add the pre-grasp motion
    robot_trajectory::RobotTrajectory grasp_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
    grasp_trajectory.setRobotTrajectoryMsg(current_state, grasp_plan.trajectory_);

    // update the current robot state
    current_state = grasp_trajectory.getLastWayPoint();

    // need to remove the object from the planning scene to avoid the collision
    current_object_ids = planning_scene_interface.getKnownObjectNames();
    if(std::find(current_object_ids.begin(), current_object_ids.end(), OBJECT_NAME) != current_object_ids.end()){
      collision_object_.operation = collision_object_.REMOVE;
      planning_scene_interface.applyCollisionObject(collision_object_);
      //planning_scene_interface.removeCollisionObjects(std::vector<std::string>{OBJECT_NAME});
    }

    // plan for the approach motion
    move_group.setStartState(current_state);
    moveit_msgs::RobotTrajectory approach_trajectory_msg;
    double fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{grasp_pose}, 0.01, 0.0, approach_trajectory_msg);

    if(fraction < 0.95){
      move_group.clearPoseTargets();
      continue;
    }
    move_group.clearPoseTargets();

    // generate the approach motion and add it to grasp trajectory
    robot_trajectory::RobotTrajectory approach_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
    approach_trajectory.setRobotTrajectoryMsg(current_state, approach_trajectory_msg);
    grasp_trajectory.append(approach_trajectory, 0.01);

    // update the current robot state
    current_state = grasp_trajectory.getLastWayPoint();

    // plan for the lift up motion
    move_group.setStartState(current_state);
    moveit_msgs::RobotTrajectory lift_up_trajectory_msg;
    fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{lift_up_pose}, 0.01, 0.0, lift_up_trajectory_msg);

    if(fraction < 0.95){
      move_group.clearPoseTargets();
      continue;
    }
    move_group.clearPoseTargets();

    // generate the lift up motion and add it to grasp trajectory
    robot_trajectory::RobotTrajectory lift_up_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
    lift_up_trajectory.setRobotTrajectoryMsg(current_state, lift_up_trajectory_msg);
    grasp_trajectory.append(lift_up_trajectory, 0.01);

    // update the current robot state
    current_state = grasp_trajectory.getLastWayPoint();

    total_trajectory.append(grasp_trajectory, 0.01);
    // visualize the grasp trajectory
    //arm_visuals->deleteAllMarkers();
    //arm_visuals->publishTrajectoryPath(grasp_trajectory);
    // arm_visuals->prompt("Press 'next' in the RvizVisualToolsGui");

    // visualize the grasp pose
    // grasp_visuals->publishEEMarkers(pre_grasp_pose, grasp_visuals->getSharedRobotState()->getJointModelGroup(END_EFFECTOR_PLANNING_GROUP), std::vector<double>{0.04,0.04}, rviz_visual_tools::BLUE, "pre-grasp_pose");
     
    foundSolution = true;
    break;
  }

  if(!foundSolution){
    std::cout << "can't find solution for grasping" << std::endl;
    return 0;
  }

  // now the object is lift up into the air, we need to test the constrained motion planning

  // calculate the initial object pose
  geometry_msgs::Pose init_object_pose = object_pose;
  init_object_pose.position.z += 0.1;

  // calculate the target object pose
  geometry_msgs::Pose rotateInZ;
  rotateInZ.orientation.x = 0; 
  rotateInZ.orientation.y = 0;
  rotateInZ.orientation.z = 1;
  rotateInZ.orientation.w = 0;

  geometry_msgs::Pose target_object_pose;
  productBetweenGeoPose(object_pose, rotateInZ, target_object_pose);
  target_object_pose.position.z += 0.1;
  target_object_pose.position.y -= 0.35;

  // calculate the target gripper pose
  geometry_msgs::Pose target_gripper_pose;
  productBetweenGeoPose(target_object_pose, current_in_hand_pose_inv, target_gripper_pose);

  // visualize the initial object pose
  targetObject_visuals->publishMesh(init_object_pose, OBJECT_MESH_FILE, rviz_visual_tools::YELLOW, 0.001, "can_object_init");
  // try to visualize the target object pose
  targetObject_visuals->publishMesh(target_object_pose, OBJECT_MESH_FILE, rviz_visual_tools::BLUE, 0.001, "can_object_target");
  targetObject_visuals->trigger();
  ros::spinOnce();

  // define the constraint on the object
  moveit_msgs::Constraints constraints;
  constraints.name = "use_equality_constraints";
  //constraints.name = "horizontal_constraints";
  constraints.in_hand_pose = current_in_hand_pose;

  // define the orientation constraint on the object
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.parameterization = moveit_msgs::OrientationConstraint::ROTATION_VECTOR;
  orientation_constraint.header.frame_id = "base_link";
  orientation_constraint.header.stamp = ros::Time(0);
  orientation_constraint.link_name = "wrist_roll_link";
  geometry_msgs::Quaternion constrained_quaternion;
  constrained_quaternion.x = 0;
  constrained_quaternion.y = 0;
  constrained_quaternion.z = 0;
  constrained_quaternion.w = 1.0;
  orientation_constraint.orientation = constrained_quaternion;
  orientation_constraint.weight = 1.0;
  orientation_constraint.absolute_x_axis_tolerance = 0.1;
  orientation_constraint.absolute_y_axis_tolerance = 0.1;
  orientation_constraint.absolute_z_axis_tolerance = 2 * 3.1415;
  constraints.orientation_constraints.push_back(orientation_constraint);

  // define the position constraint on the object
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = "base_link";
  position_constraint.header.stamp = ros::Time(0);
  position_constraint.link_name = "wrist_roll_link";
  position_constraint.weight = 1.0;

  shape_msgs::SolidPrimitive bounding_region;
  bounding_region.type = bounding_region.BOX;
  bounding_region.dimensions.resize(3);
  bounding_region.dimensions[bounding_region.BOX_X] = 2000; // for x
  bounding_region.dimensions[bounding_region.BOX_Y] = 2000; // for y 
  bounding_region.dimensions[bounding_region.BOX_Z] = 0.0005; // for z
  // If you want to use plane constraint, you should set 0.0005 for it.

  position_constraint.constraint_region.primitives.push_back(bounding_region);
  position_constraint.constraint_region.primitive_poses.push_back(target_object_pose);
  constraints.position_constraints.push_back(position_constraint);

  // set planner id
  //move_group.setPlannerId("CBIRRTConfigDefault");
  move_group.setPlannerId("CLazyPRMConfigDefault");
  move_group.setPlanningTime(1.0);
  // set action and its id
  move_group.setActionWithId("move", 1);

  // set object constraints
  move_group.setPathConstraints(constraints);

  // set the in hand pose
  move_group.setInHandPose(current_in_hand_pose);

  // set current robotstate
  move_group.setStartState(current_state);

  // add the objectt as part of arm
  // 1) add the object into the planning scene
  geometry_msgs::Pose current_object_pose;
  productBetweenGeoPose(move_group.getCurrentPose().pose, current_in_hand_pose, current_object_pose);
  collision_object_.pose = current_object_pose;
  collision_object_.operation = collision_object_.ADD;

  // 2) attach the object to the end-effector
  moveit_msgs::AttachedCollisionObject attached_collision_object_;
  attached_collision_object_.object = collision_object_;
  attached_collision_object_.link_name = std::string("wrist_roll_link");
  attached_collision_object_.touch_links = std::vector<std::string>{"l_gripper_finger_link", "r_gripper_finger_link"};
  planning_scene_interface.applyAttachedCollisionObject(attached_collision_object_);

  // set target gripper pose
  //move_group.setPoseTarget(target_gripper_pose);
  move_group.setPoseTarget(target_object_pose);

  moveit::planning_interface::MoveGroupInterface::Plan constrained_plan;
  std::vector<moveit::planning_interface::MoveGroupInterface::MotionEdge> experience;
  bool success = (move_group.plan(constrained_plan, experience) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // initialize experience graph
  ExperienceGraph experiencegraph(7, 0.01);

  int current_constraint_id = 0;

  // add returned experience to the experience graph
  for(moveit::planning_interface::MoveGroupInterface::MotionEdge edge : experience)
  {
    ExperienceGraph::Vertex v1;
    ExperienceGraph::Vertex v2;
    std::cout << "vertex 1: " << edge.verified_vertex_1_.position[0] << " "
                              << edge.verified_vertex_1_.position[1] << " "
                              << edge.verified_vertex_1_.position[2] << " "
                              << edge.verified_vertex_1_.position[3] << " "
                              << edge.verified_vertex_1_.position[4] << " "
                              << edge.verified_vertex_1_.position[5] << " "
                              << edge.verified_vertex_1_.position[6] << " " << std::endl;
    std::cout << "vertex 2: " << edge.verified_vertex_2_.position[0] << " "
                              << edge.verified_vertex_2_.position[1] << " "
                              << edge.verified_vertex_2_.position[2] << " "
                              << edge.verified_vertex_2_.position[3] << " "
                              << edge.verified_vertex_2_.position[4] << " "
                              << edge.verified_vertex_2_.position[5] << " "
                              << edge.verified_vertex_2_.position[6] << " " << std::endl;
    // std::cout << "edge weight " << edge.weight_ << std::endl;
    // add the edge to the experience graph

    // v1 = experiencegraph.addVertex(joint_values, current_constraint_id);

    // if(experiencegraph.hasVertex(current_constraint_id, edge.verified_vertex_id_2_))
    // {
    //   v2 = experiencegraph.getVertex(current_constraint_id, edge.verified_vertex_id_2_);
    // }
    // else
    // {
    //   std::vector<double> joint_values{edge.verified_vertex_2_.position[0], 
    //                                   edge.verified_vertex_2_.position[1], 
    //                                   edge.verified_vertex_2_.position[2], 
    //                                   edge.verified_vertex_2_.position[3], 
    //                                   edge.verified_vertex_2_.position[4], 
    //                                   edge.verified_vertex_2_.position[5], 
    //                                   edge.verified_vertex_2_.position[6]};
    //   v2 = experiencegraph.addVertex(joint_values, current_constraint_id, edge.verified_vertex_id_2_);
    // }

    // if(!experiencegraph.hasEdge(v1, v2)) // if the edge does not exist, then add it.
    // {
    //   experiencegraph.addEdge(v1, v2);
    // }
    
  }

  // detach object
  move_group.detachObject(collision_object_.id);
  collision_object_.operation = collision_object_.REMOVE;
  planning_scene_interface.applyCollisionObject(collision_object_);

  success = false;
  
  // if(success){
  //   std::cout << "find a solution to shift the object" << std::endl;
  //   // generate the shift_trajectory object
  //   robot_trajectory::RobotTrajectory shift_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
  //   shift_trajectory.setRobotTrajectoryMsg(current_state, constrained_plan.trajectory_);

  //   total_trajectory.append(shift_trajectory, 0.01);

  //   arm_visuals->deleteAllMarkers();
  //   arm_visuals->publishTrajectoryPath(total_trajectory);

  //   // find the shortest path
  //   // find the start state and end state
  //   std::vector<double> start_robot_state;
  //   shift_trajectory.getWayPoint(0).copyJointGroupPositions(joint_model_group, start_robot_state);

  //   ExperienceGraph::Vertex startVertex = experiencegraph.addVertex(start_robot_state, 1, 0);

  //   std::vector<double> end_robot_state;
  //   shift_trajectory.getLastWayPoint().copyJointGroupPositions(joint_model_group, end_robot_state);

  //   ExperienceGraph::Vertex endVertex = experiencegraph.addVertex(end_robot_state, 1, 1);

  //   // find the neighbors of the start and end state
  //   std::vector<ExperienceGraph::Vertex> start_neighbors = experiencegraph.getNeighbors(startVertex);

  //   // connect the start vertex to all start neighbors
  //   for(ExperienceGraph::Vertex v : start_neighbors)
  //   {
  //     if(experiencegraph.vertex_constraint_id_map[startVertex] !=  experiencegraph.vertex_constraint_id_map[v] || experiencegraph.vertex_node_id_map[startVertex] != experiencegraph.vertex_node_id_map[v])
  //     {
  //       experiencegraph.addEdge(startVertex, v);
  //     }
  //   }

  //   // find the neighbors of the end state
  //   std::vector<ExperienceGraph::Vertex> end_neighbors = experiencegraph.getNeighbors(endVertex);

  //   // connect the end vertex to all end neighbors
  //   for(ExperienceGraph::Vertex v : end_neighbors)
  //   {
  //     if(experiencegraph.vertex_constraint_id_map[endVertex] !=  experiencegraph.vertex_constraint_id_map[v] || experiencegraph.vertex_node_id_map[endVertex] != experiencegraph.vertex_node_id_map[v])
  //     {
  //       experiencegraph.addEdge(endVertex, v);
  //     }
  //   }

  //   std::vector<ExperienceGraph::Vertex> solution = experiencegraph.findSolution(startVertex, endVertex);

  //   std::cout << " solution size: " << solution.size() << std::endl;

  //   std::vector<trajectory_msgs::JointTrajectoryPoint> experience_waypoints;
  //   experience_waypoints.resize(solution.size());
  //   for(int i = 0; i < solution.size(); i++)
  //   {
  //     experience_waypoints[i].positions.resize(7);
  //     for(int j = 0; j < 7; j++)
  //     {
  //       experience_waypoints[i].positions.at(j) = experiencegraph.vertex_state_map[solution[i]][j];
  //     }
  //   }

  //   // set experience waypoints
  //   move_group.setExperience(experience_waypoints);

  //   move_group.setActionWithId("move", 1);

  //   bool success = (move_group.plan(constrained_plan, experience) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // }
  // else{
  //   std::cout << "can't find a solution to shift the object" << std::endl;
  //   return 0;
  // }



  //arm_visuals.prompt("ready to execute the plan");

  //grasp_visuals->publishEEMarkers(target_gripper_pose, grasp_visuals->getSharedRobotState()->getJointModelGroup(END_EFFECTOR_PLANNING_GROUP), std::vector<double>{0.04,0.04}, rviz_visual_tools::BLUE, "target_gripper_pose");

  return 0;
}
