#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include "manipulation_test/VisualizeGrasp.h"
#include "manipulation_test/VisualizeTable.h"
#include "manipulation_test/VisualizeObstacle.h"
#include "manipulation_test/VisualizeIntermediatePlacements.h"
#include "manipulation_test/VisualizeRegrasp.h"

#include <rail_segmentation/SearchTable.h>
#include <rail_manipulation_msgs/SegmentObjects.h>

#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "manipulation_test/task_planner.hpp"

bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(), 
        s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

void transformTFToKDL(const tf::Transform &t, KDL::Frame &k){
  for(uint i = 0; i < 3; ++i)
    k.p[i] = t.getOrigin()[i];
  for(uint i = 0; i < 9; ++i)
    k.M.data[i] = t.getBasis()[i/3][i%3]; 
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

void randomJointArray(KDL::JntArray &joint_array, const KDL::JntArray &lower_limits, const KDL::JntArray &upper_limits)
{
    for(uint j = 0; j < joint_array.data.size(); j++){
        joint_array(j) = lower_limits(j) +  (upper_limits(j) - lower_limits(j)) * ( (double)rand() / RAND_MAX);
        if(joint_array(j) > 100 or joint_array(j) < -100)
            joint_array(j) = 0.0;
    }
}

void setRobotState(moveit::core::RobotState &currentState, KDL::JntArray &random_joint_array, const std::vector<std::string> &joint_names)
{
    for(int j = 0; j < joint_names.size(); j++)
    {
        currentState.setJointPositions(joint_names[j], &random_joint_array(j));
    }
}

int main(int argc, char** argv)
{
    static const std::string OBJECT_NAME = "hammer";

    ros::init(argc, argv, "main_pipeline");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // visualize the grasp poses over the current object
    ros::ServiceClient client = node_handle.serviceClient<manipulation_test::VisualizeGrasp>("visualize_grasp");
    client.waitForExistence();

    manipulation_test::VisualizeGrasp srv;
    srv.request.grasped_object_name = OBJECT_NAME;

    std::vector<tf::Transform> grasp_transforms(3);
    std::vector<float> grasp_jawwidths(3);
    std::vector<int> grasp_types(3);

    grasp_transforms[0].setOrigin(tf::Vector3(0, 0, 0.2));
    grasp_transforms[0].setRotation(tf::Quaternion(-0.5, -0.5, 0.5, -0.5));
    grasp_jawwidths[0] = 0.08;
    grasp_types[0] = 0; // [0: lifting, 1: sliding

    grasp_transforms[1].setOrigin(tf::Vector3(0, 0.12, 0.2));
    grasp_transforms[1].setRotation(tf::Quaternion(-0.5, -0.5, 0.5, -0.5));
    grasp_jawwidths[1] = 0.08;
    grasp_types[1] = 1; // [0: lifting, 1: sliding

    grasp_transforms[2].setOrigin(tf::Vector3(0, 0.25, 0.2));
    grasp_transforms[2].setRotation(tf::Quaternion(-0.5, -0.5, 0.5, -0.5));
    grasp_jawwidths[2] = 0.08;
    grasp_types[2] = 1; // [0: lifting, 1: sliding

    for(int i = 0; i < grasp_transforms.size(); i++)
    {
        geometry_msgs::PoseStamped grasp_pose;
        transformTFToGeoPose(grasp_transforms[i], grasp_pose.pose);

        srv.request.grasp_poses.push_back(grasp_pose);
        srv.request.grasp_jawwidths.push_back(grasp_jawwidths[i]);
        srv.request.grasp_types.push_back(grasp_types[i]);
    }

    if (not client.call(srv))
    {
        ROS_ERROR("Failed to call service visualize_grasp");
        return 1;
    }

    // search for the table and add it ad the obstacle in the planning scene
    ros::ServiceClient table_client = node_handle.serviceClient<rail_segmentation::SearchTable>("table_searcher/search_table");
    table_client.waitForExistence();

    rail_segmentation::SearchTable table_srv;
    if (not table_client.call(table_srv))
    {
        ROS_ERROR("Failed to call service search_table");
        return 1;
    }

    // visualize the table
    ros::ServiceClient table_visualizer = node_handle.serviceClient<manipulation_test::VisualizeTable>("visualize_table");
    table_visualizer.waitForExistence();

    manipulation_test::VisualizeTable table_visualize_srv;
    table_visualize_srv.request.width = table_srv.response.width;
    table_visualize_srv.request.depth = table_srv.response.depth;
    table_visualize_srv.request.height = 0.001;
    table_visualize_srv.request.center = table_srv.response.center;
    table_visualize_srv.request.orientation = table_srv.response.orientation;

    if (not table_visualizer.call(table_visualize_srv))
    {
        ROS_ERROR("Failed to call service visualize_table");
        return 1;
    }

    // search fo the obstacle on the table
    ros::ServiceClient obstacle_client = node_handle.serviceClient<rail_manipulation_msgs::SegmentObjects>("table_searcher/segment_objects");
    obstacle_client.waitForExistence();

    rail_manipulation_msgs::SegmentObjects obstacle_srv;

    if (not obstacle_client.call(obstacle_srv))
    {
        ROS_ERROR("Failed to call service segment_objects");
        return 1;
    }

    if(obstacle_srv.response.segmented_objects.objects.size() == 0)
    {
        ROS_INFO("No obstacle found on the table");
        return 0;
    }

    // // visualize the obstacle
    ros::ServiceClient obstacle_visualizer = node_handle.serviceClient<manipulation_test::VisualizeObstacle>("visualize_obstacle");
    obstacle_visualizer.waitForExistence();

    std::vector<bool> is_target(obstacle_srv.response.segmented_objects.objects.size(), false);

    manipulation_test::VisualizeObstacle obstacle_visualize_srv;
    for(int i = 0; i < obstacle_srv.response.segmented_objects.objects.size(); i++)
    {
        obstacle_visualize_srv.request.widths.push_back(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.y);
        obstacle_visualize_srv.request.depths.push_back(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.x);
        obstacle_visualize_srv.request.heights.push_back(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.z);
        obstacle_visualize_srv.request.obstacle_poses.push_back(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose);
        obstacle_visualize_srv.request.obstacle_ids.push_back(i);
        obstacle_visualize_srv.request.is_target.push_back(is_target[i]);
    }

    if (not obstacle_visualizer.call(obstacle_visualize_srv))
    {
        ROS_ERROR("Failed to call service visualize_obstacle");
        return 1;
    }

    std::cout << "Please enter the grasped object id: " << std::endl;
    std::string grasped_object_id_str;
    std::getline(std::cin, grasped_object_id_str);
    
    while(is_number(grasped_object_id_str) == false or std::stoi(grasped_object_id_str) >= obstacle_srv.response.segmented_objects.objects.size() or std::stoi(grasped_object_id_str) < 0)
    {
        std::cout << "You should enter a integer and less than the obstacle size number and larger or equal to 0!! " << std::endl;
        std::getline(std::cin, grasped_object_id_str);
    }

    std::cout << "grasped object id: " << grasped_object_id_str << std::endl;
    int grasped_object_id = std::stoi(grasped_object_id_str);
    is_target[grasped_object_id] = true;

    // initialize the collision environment
    
    using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryf>;
    using CollisionObjectPtr_t = std::shared_ptr<fcl::CollisionObjectf>;
    std::vector<CollisionObjectPtr_t> collision_objects;
    CollisionObjectPtr_t target_collision_object;

    for(int i = 0; i < obstacle_srv.response.segmented_objects.objects.size(); i++)
    {
        obstacle_visualize_srv.request.is_target[i] = is_target[i];
        if(is_target[i] == false)
        {
            // create the collision object with only geometry
            CollisionGeometryPtr_t box_geom = std::make_shared<fcl::Boxf>(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.x, 
                                                                          obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.y, 
                                                                          obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.z);
            CollisionObjectPtr_t collision_object = std::make_shared<fcl::CollisionObjectf>(box_geom);

            // set the collision object pose
            fcl::Quaternionf box_quaternion = fcl::Quaternionf(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.orientation.w, 
                                                               obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.orientation.x, 
                                                               obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.orientation.y, 
                                                               obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.orientation.z);
            fcl::Vector3f box_position = fcl::Vector3f(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.x, 
                                                       obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.y, 
                                                       obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.z);
            collision_object->setTransform(box_quaternion, box_position);
            
            // add the collision object to the collision environment
            collision_objects.push_back(collision_object);
        }
        else
        {
            // create the collision object with only geometry
            CollisionGeometryPtr_t box_geom = std::make_shared<fcl::Boxf>(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.x, 
                                                                          obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.y, 
                                                                          obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.z);
            target_collision_object = std::make_shared<fcl::CollisionObjectf>(box_geom);
        }
    }

    if (not obstacle_visualizer.call(obstacle_visualize_srv))
    {
        ROS_ERROR("Failed to call service visualize_obstacle");
        return 1;
    }

    // convert the table point cloud to pcl point cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(table_srv.response.point_cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> table_point_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2,table_point_cloud);
    
    // get the table transform
    tf::Transform table_transform(tf::Quaternion(table_srv.response.orientation.x, table_srv.response.orientation.y, table_srv.response.orientation.z, table_srv.response.orientation.w), 
                                  tf::Vector3(table_srv.response.center.x, table_srv.response.center.y, table_srv.response.center.z));

    // get the target object transform
    tf::Transform target_object_transform(tf::Quaternion(obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.orientation.x, 
                                                        obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.orientation.y, 
                                                        obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.orientation.z, 
                                                        obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.orientation.w), 
                                          tf::Vector3(obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.position.x, 
                                                      obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.position.y, 
                                                      obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.position.z));

    std::vector<tf::Transform> actual_grasp_transforms;
    for(int i = 0; i < srv.response.actual_grasp_poses.size(); i++)
    {
        tf::Transform actual_grasp_transform = target_object_transform.inverse() * tf::Transform(tf::Quaternion(srv.response.actual_grasp_poses[i].pose.orientation.x, 
                                                                                                                srv.response.actual_grasp_poses[i].pose.orientation.y, 
                                                                                                                srv.response.actual_grasp_poses[i].pose.orientation.z, 
                                                                                                                srv.response.actual_grasp_poses[i].pose.orientation.w), 
                                                                                                tf::Vector3(srv.response.actual_grasp_poses[i].pose.position.x, 
                                                                                                            srv.response.actual_grasp_poses[i].pose.position.y, 
                                                                                                            srv.response.actual_grasp_poses[i].pose.position.z));
        actual_grasp_transforms.push_back(actual_grasp_transform);
    }

    // get the target object transform in the table frame
    tf::Transform target_object_transform_in_table_frame = table_transform.inverse() * target_object_transform;

    // get a set of random target object poses in the table frame.
    std::vector<tf::Transform> random_target_object_transforms;
    for(int i = 0; i < 50; i++)
    {
        int index = rand() % table_point_cloud.size();
        tf::Vector3 random_point(table_point_cloud.points[index].x, table_point_cloud.points[index].y, table_point_cloud.points[index].z);
        // get random point in the table frame
        tf::Vector3 random_point_in_table_frame = table_transform.inverse() * random_point;

        tf::Transform random_target_object_transform_in_table_frame(target_object_transform_in_table_frame);
        random_target_object_transform_in_table_frame.setOrigin(tf::Vector3(random_point_in_table_frame.x(), random_point_in_table_frame.y(), target_object_transform_in_table_frame.getOrigin().z()));
        random_target_object_transform_in_table_frame.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), (rand() % 360) * M_PI / 180.0) * random_target_object_transform_in_table_frame.getRotation());

        tf::Transform random_target_object_transform = table_transform * random_target_object_transform_in_table_frame;

        // set the target collision object transform
        fcl::Quaternionf box_quaternion = fcl::Quaternionf(random_target_object_transform.getRotation().w(), 
                                                           random_target_object_transform.getRotation().x(), 
                                                           random_target_object_transform.getRotation().y(), 
                                                           random_target_object_transform.getRotation().z());
        fcl::Vector3f box_position = fcl::Vector3f(random_target_object_transform.getOrigin().x(), 
                                                   random_target_object_transform.getOrigin().y(), 
                                                   random_target_object_transform.getOrigin().z());
        target_collision_object->setTransform(box_quaternion, box_position);

        bool is_collision = false;
        for(int j = 0; j < collision_objects.size(); j++)
        {
            // set the collision request structure, here we just use the default setting
            fcl::CollisionRequestf request;
            // result will be returned via the collision result structure
            fcl::CollisionResultf result;
            // perform collision test
            fcl::collide(collision_objects[j].get(), target_collision_object.get(), request, result);
            if(result.isCollision())
            {
                is_collision = true;
                break;
            }
        }

        if(is_collision)
        {
            // i--;
            continue;
        }
        
        random_target_object_transforms.push_back(random_target_object_transform);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // At this point, we have both grasp poses over the object and a set of possible intermediate object placements for re-grasping on the table. //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // based on the above information, we use them to initialize the task planner.

    // initialize the moveit interfaces

    static const std::string PLANNING_GROUP = "arm";
    static const std::string END_EFFECTOR_PLANNING_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface end_effector_move_group(END_EFFECTOR_PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    // get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

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
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver);

    KDL::JntArray nominal(chain.getNrOfJoints());
    for(uint j = 0; j < nominal.data.size(); j++){
        nominal(j) = (ll(j) + ul(j)) / 2.0;
    }

    // initialize varaibles to find all feasible re-grasp poses and intermedaite placements
    // the following information will be used to initialize the task planner
    std::vector<tf::Transform> feasible_intermediate_placement_transforms;
    std::vector<std::vector<tf::Transform>> feasible_regrasp_transforms;
    std::vector<std::vector<int>> regrasp_ids;
    std::vector<std::vector<int>> regrasp_types;
    std::vector<std::vector<std::vector<moveit_msgs::RobotTrajectory>>> lifting_motions;

    srand(time(0));
    KDL::JntArray random_joint_array(chain.getNrOfJoints());
    moveit::core::RobotState currentState = *(move_group.getCurrentState());
    std::vector<std::string> joint_names = move_group.getActiveJoints();

    // find all feasible re-grasp poses
    for(tf::Transform intermedaite_placement_transform: random_target_object_transforms)
    {
        int number_of_feable_grasp_poses = 0;
        std::vector<bool> is_feasible_grasp_poses(actual_grasp_transforms.size(), false);
        for(int i = 0; i < actual_grasp_transforms.size(); i++)
        {
            // check whether the grasp pose is feasible
            KDL::Frame end_effector_pose;
            KDL::JntArray result = nominal;
            transformTFToKDL(intermedaite_placement_transform * actual_grasp_transforms[i], end_effector_pose);
            if(kdl_solver.CartToJnt(result, end_effector_pose, result) >= 0)
            {
                is_feasible_grasp_poses[i] = true;
                number_of_feable_grasp_poses++;
            }
        }

        if(number_of_feable_grasp_poses < 2) // there are less than two feasible grasp poses in this intermediate placement for re-grasping.
        {
            continue;
        }

        // find all feasible re-grasp poses with its lifting motion in this intermediate placement
        std::vector<tf::Transform> feasible_regrasp_transforms_temp;  // feasible re-grasp poses in this intermediate placement
        std::vector<int> regrasp_ids_temp; // the id of re-grasp poses in this intermediate placement
        std::vector<int> regrasp_types_temp; // the type of re-grasp poses in this intermediate placement
        std::vector<std::vector<moveit_msgs::RobotTrajectory>> lifting_motions_temp; // the lifting motions of feasible re-grasp poses in this intermediate placement
        
        // search all state in this intermediate placement
        for(int g = 0; g < actual_grasp_transforms.size(); g++)
        {
            if(!is_feasible_grasp_poses[g]) // this grasp pose is not feasible so skip it.
                continue;

            // find pre-grasp pose and transform of current grasp pose.
            tf::Transform pre_grasp_pose_in_world_frame = intermedaite_placement_transform * actual_grasp_transforms[g] * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.04, 0, 0));
            geometry_msgs::Pose pre_grasp_pose;
            transformTFToGeoPose(pre_grasp_pose_in_world_frame, pre_grasp_pose);

            std::vector<moveit_msgs::RobotTrajectory> current_lifting_motions; // the lifting motions of current grasp pose
            // desired end-effector pose
            KDL::Frame end_effector_pose;
            transformTFToKDL(intermedaite_placement_transform * actual_grasp_transforms[g], end_effector_pose);

            for(int k = 0; k < 10; k++)
            {

                if(k == 0)
                    random_joint_array = nominal;
                else
                    randomJointArray(random_joint_array, ll, ul);
                
                // check whether the arm configuration is feasible
                if(kdl_solver.CartToJnt(random_joint_array, end_effector_pose, random_joint_array) >= 0)
                {
                    // check whether this arm configuration can be used to lift the object.
                    setRobotState(currentState, random_joint_array, joint_names);

                    move_group.setStartState(currentState);
                    moveit_msgs::RobotTrajectory approach_trajectory_msg;

                    double fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{pre_grasp_pose}, 0.03, 5.0, approach_trajectory_msg, false);

                    if(fraction >= 0.95)
                        current_lifting_motions.push_back(approach_trajectory_msg);
                    move_group.clearPoseTargets();
                }

                if(current_lifting_motions.size() > 2) // no need to find more lifting motions
                    break;
            }

            if(current_lifting_motions.size() > 0) // has a way to lift up the object with current grasp pose 
            {
                lifting_motions_temp.push_back(current_lifting_motions);
                feasible_regrasp_transforms_temp.push_back(pre_grasp_pose_in_world_frame);
                regrasp_ids_temp.push_back(g);
                regrasp_types_temp.push_back(grasp_types[g]);
            }
        }
        if(feasible_regrasp_transforms_temp.size() > 1) // has at least two feasible re-grasp poses in this intermediate placement
        {
            feasible_intermediate_placement_transforms.push_back(intermedaite_placement_transform);
            feasible_regrasp_transforms.push_back(feasible_regrasp_transforms_temp);
            regrasp_ids.push_back(regrasp_ids_temp);
            regrasp_types.push_back(regrasp_types_temp);
            lifting_motions.push_back(lifting_motions_temp);
        }
    }

    // try to add the current object pose as one of the intermediate placement
    std::vector<tf::Transform> feasible_regrasp_transforms_for_init;  // feasible re-grasp poses in this intermediate placement
    std::vector<int> regrasp_ids_for_init; // the id of re-grasp poses in this intermediate placement
    std::vector<int> regrasp_types_for_init; // the type of re-grasp poses in this intermediate placement
    std::vector<std::vector<moveit_msgs::RobotTrajectory>> lifting_motions_for_init; // the lifting motions of feasible re-grasp poses in this intermediate placement

    for(int g = 0; g < actual_grasp_transforms.size(); g++)
    {
        tf::Transform pre_grasp_pose_in_world_frame = target_object_transform * actual_grasp_transforms[g] * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.04, 0, 0));
        geometry_msgs::Pose pre_grasp_pose;
        transformTFToGeoPose(pre_grasp_pose_in_world_frame, pre_grasp_pose);

        std::vector<moveit_msgs::RobotTrajectory> current_lifting_motions; // the lifting motions of current grasp pose
        // desired end-effector pose
        KDL::Frame end_effector_pose;
        transformTFToKDL(target_object_transform * actual_grasp_transforms[g], end_effector_pose);

        for(int k = 0; k < 30; k++)
        {

            if(k == 0)
                random_joint_array = nominal;
            else
                randomJointArray(random_joint_array, ll, ul);
            
            // check whether the arm configuration is feasible
            if(kdl_solver.CartToJnt(random_joint_array, end_effector_pose, random_joint_array) >= 0)
            {
                // check whether this arm configuration can be used to lift the object.
                setRobotState(currentState, random_joint_array, joint_names);

                move_group.setStartState(currentState);
                moveit_msgs::RobotTrajectory approach_trajectory_msg;

                double fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{pre_grasp_pose}, 0.03, 5.0, approach_trajectory_msg, false);

                if(fraction >= 0.95)
                    current_lifting_motions.push_back(approach_trajectory_msg);
                move_group.clearPoseTargets();
            }

            if(current_lifting_motions.size() > 2) // no need to find more lifting motions
                break;
        }

        if(current_lifting_motions.size() > 0)
        { // this grasp pose can be used to lift the object.
            lifting_motions_for_init.push_back(current_lifting_motions);
            feasible_regrasp_transforms_for_init.push_back(pre_grasp_pose_in_world_frame);
            regrasp_ids_for_init.push_back(g);
            regrasp_types_for_init.push_back(grasp_types[g]);
        }
    }

    if(feasible_regrasp_transforms_for_init.size() > 0)
    {
        feasible_intermediate_placement_transforms.push_back(target_object_transform);
        feasible_regrasp_transforms.push_back(feasible_regrasp_transforms_for_init);
        regrasp_ids.push_back(regrasp_ids_for_init);
        regrasp_types.push_back(regrasp_types_for_init);
        lifting_motions.push_back(lifting_motions_for_init);
    }
    else
    {
        std::cout << "no way to grasp the object" << std::endl;
        return 0;
    }

    moveit::core::RobotState current_state = *(move_group.getCurrentState());

    int num_of_lifting_grasps = 0;

    //init the task planner.
    TaskPlanner task_planner(std::vector<unsigned int>{feasible_intermediate_placement_transforms.size()}, std::vector<unsigned int>{actual_grasp_transforms.size()});

    // add all action motions into the task planner.
    for(int f = 0; f < 1; f++)
    {
        for(int p = 0; p < feasible_intermediate_placement_transforms.size(); p++)
        {
            for(int g = 0; g < regrasp_ids[p].size(); g++)
            {
                if(regrasp_types[p][g] == 0) // this grasp pose can be used to lift the object.
                {
                    num_of_lifting_grasps++;
                    for(int k = 0; k < lifting_motions[p][g].size(); k++)
                    {
                        task_planner.addActionBetweenFoliations(lifting_motions[p][g][k], f, regrasp_ids[p][g], f+1, 0, 1.0);
                    }
                }
                for(int k = 0; k < lifting_motions[p][g].size(); k++)
                {
                    task_planner.addActionBetweenManifolds(lifting_motions[p][g][k], p, regrasp_ids[p][g], f, 0.0);
                }
            }
        }
    }

    if(num_of_lifting_grasps == 0)
    {
        std::cout << "no way to lift the object" << std::endl;
        return 0;
    }

    task_planner.constructMDPGraph();

    task_planner.policyIteration();

    // visualize the regrasp poses
    ros::ServiceClient regrasp_poses_visualizer = node_handle.serviceClient<manipulation_test::VisualizeRegrasp>("visualize_regrasp");
    regrasp_poses_visualizer.waitForExistence();

    manipulation_test::VisualizeRegrasp regrasp_poses_visualize_srv;
    for(int i = 0; i < feasible_regrasp_transforms.size(); i++)
    {
        for(int j = 0; j < feasible_regrasp_transforms[i].size(); j++)
        {
            geometry_msgs::PoseStamped grasp_pose;
            transformTFToGeoPose(feasible_regrasp_transforms[i][j], grasp_pose.pose);
            regrasp_poses_visualize_srv.request.grasp_poses.push_back(grasp_pose);
            regrasp_poses_visualize_srv.request.grasp_jawwidths.push_back(0.08);
            regrasp_poses_visualize_srv.request.grasp_types.push_back(regrasp_types[i][j]);
        }
    }

    if (!regrasp_poses_visualizer.call(regrasp_poses_visualize_srv))
    {
        ROS_ERROR("Failed to call service visualize_regrasp");
        return 1;
    }


    // visualize the random target object poses
    ros::ServiceClient intermediate_placements_visualizer = node_handle.serviceClient<manipulation_test::VisualizeIntermediatePlacements>("visualize_intermediate_placements");
    intermediate_placements_visualizer.waitForExistence();

    manipulation_test::VisualizeIntermediatePlacements intermediate_placements_visualize_srv;
    for(int i = 0; i < feasible_intermediate_placement_transforms.size(); i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = feasible_intermediate_placement_transforms[i].getOrigin().x();
        pose_stamped.pose.position.y = feasible_intermediate_placement_transforms[i].getOrigin().y();
        pose_stamped.pose.position.z = feasible_intermediate_placement_transforms[i].getOrigin().z();
        pose_stamped.pose.orientation.x = feasible_intermediate_placement_transforms[i].getRotation().x();
        pose_stamped.pose.orientation.y = feasible_intermediate_placement_transforms[i].getRotation().y();
        pose_stamped.pose.orientation.z = feasible_intermediate_placement_transforms[i].getRotation().z();
        pose_stamped.pose.orientation.w = feasible_intermediate_placement_transforms[i].getRotation().w();
        intermediate_placements_visualize_srv.request.intermediate_placement_poses.push_back(pose_stamped);

        intermediate_placements_visualize_srv.request.intermediate_placement_depths.push_back(obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.x);
        intermediate_placements_visualize_srv.request.intermediate_placement_widths.push_back(obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.y);
        intermediate_placements_visualize_srv.request.intermediate_placement_heights.push_back(obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.z);
    }

    if(!intermediate_placements_visualizer.call(intermediate_placements_visualize_srv))
    {
        ROS_ERROR("Failed to call service visualize_intermediate_placements");
        return 1;
    }

    return 0;
}