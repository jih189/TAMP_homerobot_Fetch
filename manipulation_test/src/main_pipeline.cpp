#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetStateValidity.h>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include "manipulation_test/VisualizeGrasp.h"
#include "manipulation_test/VisualizeTable.h"
#include "manipulation_test/VisualizeObstacle.h"
#include "manipulation_test/VisualizeIntermediatePlacements.h"
#include "manipulation_test/VisualizeRegrasp.h"
#include "manipulation_test/VisualizeCom.h"

#include <rail_segmentation/SearchTable.h>
#include <rail_manipulation_msgs/SegmentObjects.h>
#include <sensor_msgs/CameraInfo.h>

#include <ros_tensorflow_msgs/Predict.h>
#include <ros_tensorflow_msgs/ComPredict.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "manipulation_test/task_planner.hpp"

#include <cmath>

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

void GeoPoseTotransformTF(const geometry_msgs::Pose &p, tf::Transform &t){
  t.setOrigin(tf::Vector3(p.position.x, p.position.y, p.position.z));
  t.setRotation(tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w));
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

bool lift_torque_test(tf::Vector3 object_mass_center, float object_mass, tf::Stamped<tf::Transform> grasp_pose)
{
    float grasping_force = 3.5;
    float friction_coefficient = 0.01;
    float pi_constant = 3.141592653589793238462643383279502884197169399375;

    tf::Matrix3x3 grasp_rotation = grasp_pose.getBasis();
    tf::Vector3 y_grasp_axis = grasp_rotation.getColumn(1);
    tf::Vector3 gravity_axis(0.0,0.0,-1.0);
    float angle_between_grasping_axis_and_gravity_axis = y_grasp_axis.angle(gravity_axis);
    angle_between_grasping_axis_and_gravity_axis = std::min(angle_between_grasping_axis_and_gravity_axis, pi_constant - angle_between_grasping_axis_and_gravity_axis);
    if( angle_between_grasping_axis_and_gravity_axis == 0.0) // when the grasping axis is parallel to the gravity axis, then the torque is zero.
        return true;

    tf::Vector3 horizotal_line = tf::Vector3(y_grasp_axis.x(), y_grasp_axis.y(), 0.0).cross(tf::Vector3(0.0,0.0,1.0));

    tf::Vector3 grasping_center_point = grasp_pose * tf::Vector3(0.17,0.0,0.0);
    tf::Vector3 grasping_point_in_line = object_mass_center - grasping_center_point;
    float horizontal_distance = std::abs(grasping_point_in_line.dot(horizotal_line)/horizotal_line.length());

    if(object_mass * horizontal_distance * std::sin(angle_between_grasping_axis_and_gravity_axis) <= 
                (object_mass * 9.8 * std::cos(angle_between_grasping_axis_and_gravity_axis) + grasping_force) * friction_coefficient)
        return true;
    else
        return false;
}

bool is_in_view(tf::Vector3 point_3d, double fx, double fy, double tx, double ty, double camera_width, double camera_height)
{
    double x = point_3d.x();
    double y = point_3d.y();
    double z = point_3d.z();

    double u = fx * x / z + tx;
    double v = fy * y / z + ty;

    if(u < 0 or u > camera_width or v < 0 or v > camera_height)
        return false;
    else
        return true;
}

void generatePointCloudOfPlane(tf::Transform planePose, double planeWidth, double planeLength, double planeHeight, double planeResolution, sensor_msgs::PointCloud2 &planePointCloud)
{
    // generate PointCloud2 of the plane
    pcl::PointCloud<pcl::PointXYZ> plane_cloud;
    plane_cloud.width = planeWidth / planeResolution;
    plane_cloud.height = planeLength / planeResolution;
    plane_cloud.is_dense = false;
    plane_cloud.points.resize(plane_cloud.width * plane_cloud.height);

    for (size_t i = 0; i < plane_cloud.points.size(); ++i)
    {
        plane_cloud.points[i].x = - planeHeight / 2.0;
        plane_cloud.points[i].y = (i % (int)plane_cloud.width) * planeResolution - planeWidth / 2.0;
        plane_cloud.points[i].z = (i / (int)plane_cloud.width) * planeResolution - planeLength / 2.0;
    }

    // convert tf::Transform to Eigen::Affine3d
    Eigen::Affine3d plane_pose;
    tf::transformTFToEigen(planePose, plane_pose);

    // transform the point cloud
    pcl::transformPointCloud(plane_cloud, plane_cloud, plane_pose);

    pcl::toROSMsg(plane_cloud, planePointCloud);
    planePointCloud.header.frame_id = "base_link";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_pipeline");
    bool param_bool;
    float param_float;
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    //////////////////////////////////////////
    bool use_regrasp = true; // if true, the regrasp is used.
    bool is_execute = true; // if true, the robot is executed.
    bool re_analyze = true; // if true, replanning will be run for each re-grasping.
    float obj_mass = 1.0; // expected mass of the object to be grasped
    //////////////////////////////////////////
    if (private_node_handle.getParam("use_regrasp", param_bool))
    {
        use_regrasp = param_bool;
    }
    if (private_node_handle.getParam("is_execute", param_bool))
    {
        is_execute = param_bool;
    }
    if (private_node_handle.getParam("re_analyze", param_bool))
    {
        re_analyze = param_bool;
    }
    if (private_node_handle.getParam("obj_mass", param_float))
    {
        obj_mass = param_float;
    } 


    if(!is_execute) // re-analyze must be used when is_execute is true.
        re_analyze = false;

    std::cout<< "main_pipeline configuration" << std::endl;
    std::cout << "use regrasp: " << use_regrasp << std::endl;
    std::cout << "is execute: " << is_execute << std::endl;
    std::cout << "re analyze: " << re_analyze << std::endl;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // initialize the moveit interfaces
    static const std::string PLANNING_GROUP = "arm";
    static const std::string END_EFFECTOR_PLANNING_GROUP = "gripper";
    static const std::string ARM_HAND_PLANNING_GROUP = "arm_with_gripper";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface end_effector_move_group(END_EFFECTOR_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface arm_hand_move_group(ARM_HAND_PLANNING_GROUP);

    // get all joint names
    std::vector<std::string> joint_names = move_group.getActiveJoints();
    std::vector<std::string> finger_joint_names = end_effector_move_group.getActiveJoints();

    // init planning scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   
    // get joint model group
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const robot_state::JointModelGroup* end_effector_joint_model_group = end_effector_move_group.getCurrentState()->getJointModelGroup(END_EFFECTOR_PLANNING_GROUP);

    // get the robot kinematic model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    std::vector<double> home_joint_values = {-0.698, 1.34, -2.39, -1.62, 0.087, -1.64, 1.34};

    // std::vector<double> open_finger_joint_values = {0.05, 0.05};
    // end_effector_move_group.setJointValueTarget(finger_joint_names, open_finger_joint_values);
    // end_effector_move_group.move();
    // ros::Duration(1.0).sleep();

    // std::vector<double> close_finger_joint_values = {0.027, 0.027};
    // end_effector_move_group.setJointValueTarget(finger_joint_names, close_finger_joint_values);
    // end_effector_move_group.move();
    // ros::Duration(1.0).sleep();

    // return 0;


    // need to get necessary transform information vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    // init the tf listener 
    tf::TransformListener listener;

    // need to get the should transform for ik solver
    tf::StampedTransform torso_transform;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/base_link", "/torso_lift_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/base_link","/torso_lift_link", ros::Time(0), torso_transform);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    // need to get the camera pose as well.
    tf::StampedTransform camera_transform;
    geometry_msgs::TransformStamped camera_stamped_transform;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/base_link", "/head_camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/base_link","/head_camera_rgb_optical_frame", ros::Time(0), camera_transform);
        tf::transformStampedTFToMsg(camera_transform, camera_stamped_transform);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    // get camera info
    sensor_msgs::CameraInfo camera_info;
    camera_info = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/head_camera/rgb/camera_info", node_handle));
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // init tracik solver vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
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

    srand(time(0));
    KDL::JntArray random_joint_array(chain.getNrOfJoints());
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // initialize the ros server clients vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    // init the state validity checker for the robot
    ros::ServiceClient validity_client =  node_handle.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");
    validity_client.waitForExistence();
    ros::ServiceClient table_client = node_handle.serviceClient<rail_segmentation::SearchTable>("table_searcher/search_table");
    table_client.waitForExistence();
    ros::ServiceClient table_visualizer = node_handle.serviceClient<manipulation_test::VisualizeTable>("visualize_table");
    table_visualizer.waitForExistence();
    // init the obstacle searcher.
    ros::ServiceClient obstacle_client = node_handle.serviceClient<rail_manipulation_msgs::SegmentObjects>("table_searcher/segment_objects");
    obstacle_client.waitForExistence();
    ros::ServiceClient obstacle_visualizer = node_handle.serviceClient<manipulation_test::VisualizeObstacle>("visualize_obstacle");
    obstacle_visualizer.waitForExistence();
    // init client for grasp prediction.
    ros::ServiceClient grasp_prediction_client = node_handle.serviceClient<ros_tensorflow_msgs::Predict>("grasp_predict");
    grasp_prediction_client.waitForExistence();
    // init client for center of mass prediction.
    ros::ServiceClient com_prediction_client = node_handle.serviceClient<ros_tensorflow_msgs::ComPredict>("CoMPredict");
    com_prediction_client.waitForExistence();
    // show the grasp prediction result
    ros::ServiceClient grasp_poses_visualizer = node_handle.serviceClient<manipulation_test::VisualizeRegrasp>("visualize_regrasp");
    grasp_poses_visualizer.waitForExistence();
    // visualize the random target object poses
    ros::ServiceClient intermediate_placements_visualizer = node_handle.serviceClient<manipulation_test::VisualizeIntermediatePlacements>("visualize_intermediate_placements");
    intermediate_placements_visualizer.waitForExistence();
    // visualize the center of mass prediction
    ros::ServiceClient com_predict_visualizer = node_handle.serviceClient<manipulation_test::VisualizeCom>("visualize_point_cloud");
    com_predict_visualizer.waitForExistence();
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    bool hasTargetObject = false;
    tf::Transform target_object_transform;

    int num_of_trials = 5;
    bool retry = false;

    //************************************* the main loop to manipulate the object ****************************************************************//
    do
    {
        // init the wall as the collision object.
        moveit_msgs::CollisionObject collision_object_wall;
        collision_object_wall.header.frame_id = move_group.getPlanningFrame();
        collision_object_wall.id = "wall";
        shape_msgs::SolidPrimitive wall_primitive;
        wall_primitive.type = wall_primitive.BOX;
        wall_primitive.dimensions.resize(3);
        wall_primitive.dimensions[0] = 2.0;
        wall_primitive.dimensions[1] = 0.1;
        wall_primitive.dimensions[2] = 2.0;

        geometry_msgs::Pose wall_pose_right;
        wall_pose_right.position.x = 0.0;
        wall_pose_right.position.y = -0.8;
        wall_pose_right.position.z = 0.5;

        geometry_msgs::Pose wall_pose_left;
        wall_pose_left.position.x = 0.0;
        wall_pose_left.position.y = 0.8;
        wall_pose_left.position.z = 0.5;

        collision_object_wall.primitives.push_back(wall_primitive);
        collision_object_wall.primitive_poses.push_back(wall_pose_right);

        collision_object_wall.primitives.push_back(wall_primitive);
        collision_object_wall.primitive_poses.push_back(wall_pose_left);
        // add the wall to the planning scene
        collision_object_wall.operation = collision_object_wall.ADD;
        planning_scene_interface.applyCollisionObject(collision_object_wall);

        retry = false;
        // search and visualize the table, and prepare it for the planning scene vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        rail_segmentation::SearchTable table_srv;
        if (not table_client.call(table_srv))
        {
            ROS_ERROR("Failed to call service search_table");
            return 1;
        }

        // get the table transform
        tf::Transform table_transform(tf::Quaternion(table_srv.response.orientation.x, table_srv.response.orientation.y, table_srv.response.orientation.z, table_srv.response.orientation.w), 
                                    tf::Vector3(table_srv.response.center.x, table_srv.response.center.y, table_srv.response.center.z));

        // visualize the table
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

        // init the table as the collision object.
        moveit_msgs::CollisionObject collision_object_table;
        collision_object_table.header.frame_id = move_group.getPlanningFrame();
        collision_object_table.id = "table";
        shape_msgs::SolidPrimitive table_primitive;
        table_primitive.type = table_primitive.BOX;
        table_primitive.dimensions.resize(3);
        table_primitive.dimensions[0] = table_srv.response.depth;
        table_primitive.dimensions[1] = table_srv.response.width;
        table_primitive.dimensions[2] =  0.005;

        shape_msgs::SolidPrimitive table_base_primitive;
        table_base_primitive.type = table_primitive.BOX;
        table_base_primitive.dimensions.resize(3);
        table_base_primitive.dimensions[0] = table_srv.response.depth;
        table_base_primitive.dimensions[1] = table_srv.response.width;
        table_base_primitive.dimensions[2] =  0.5;

        geometry_msgs::Pose table_pose;
        table_pose.position.x = table_srv.response.center.x;
        table_pose.position.y = table_srv.response.center.y;
        table_pose.position.z = table_srv.response.center.z;
        table_pose.orientation.x = table_srv.response.orientation.x;
        table_pose.orientation.y = table_srv.response.orientation.y;
        table_pose.orientation.z = table_srv.response.orientation.z;
        table_pose.orientation.w = table_srv.response.orientation.w;

        geometry_msgs::Pose table_primitive_pose;
        table_primitive_pose.position.x = 0.0;
        table_primitive_pose.position.y = 0.0;
        table_primitive_pose.position.z = 0.0;

        collision_object_table.primitives.push_back(table_primitive);
        collision_object_table.primitive_poses.push_back(table_primitive_pose);

        geometry_msgs::Pose table_base_primitive_pose;
        table_base_primitive_pose.position.x = 0.0;
        table_base_primitive_pose.position.y = 0.0;
        table_base_primitive_pose.position.z = -.25;
        collision_object_table.primitives.push_back(table_base_primitive);
        collision_object_table.primitive_poses.push_back(table_base_primitive_pose);

        collision_object_table.pose = table_pose;
        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        // search and visualize objects on the table. vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        rail_manipulation_msgs::SegmentObjects obstacle_srv;
        if (not obstacle_client.call(obstacle_srv))
        {
            ROS_ERROR("Failed to call service segment_objects");
            return 1;
        }
        if(obstacle_srv.response.segmented_objects.objects.size() == 0)
        {
            ROS_INFO("No object found on the table");
            return 0;
        }
        
        // visualize the obstacle
        manipulation_test::VisualizeObstacle obstacle_visualize_srv;
        for(int i = 0; i < obstacle_srv.response.segmented_objects.objects.size(); i++)
        {
            obstacle_visualize_srv.request.widths.push_back(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.y);
            obstacle_visualize_srv.request.depths.push_back(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.x);
            obstacle_visualize_srv.request.heights.push_back(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.z);
            obstacle_visualize_srv.request.obstacle_poses.push_back(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose);
            obstacle_visualize_srv.request.obstacle_ids.push_back(i);
            obstacle_visualize_srv.request.is_target.push_back(false);
        }
        if (not obstacle_visualizer.call(obstacle_visualize_srv))
        {
            ROS_ERROR("Failed to call service visualize_obstacle");
            return 1;
        }
        
        // print out the obstacle position with each ID
        for(int i = 0; i < obstacle_srv.response.segmented_objects.objects.size(); i++)
        {
            std::cout << i << " obstacle position is: ";
            std::cout << "[ " << obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.x;
            std::cout << ", " << obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.y;
            std::cout << ", " << obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.z << " ]" << std::endl;
        }
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        
        // need to decide which object is the target object vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        // init a vector to indicate whether the object is the target object
        // std::vector<bool> is_target(obstacle_srv.response.segmented_objects.objects.size(), false);
        int grasped_object_id;

        if(!hasTargetObject) // if the target object is not selected, then select the target object.
        {
            std::cout << "Please enter the grasped object id: " << std::endl;
            std::string grasped_object_id_str;
            std::getline(std::cin, grasped_object_id_str);
            
            while(is_number(grasped_object_id_str) == false or std::stoi(grasped_object_id_str) >= obstacle_srv.response.segmented_objects.objects.size() or std::stoi(grasped_object_id_str) < 0)
            {
                std::cout << "You should enter a integer and less than the obstacle size number and larger or equal to 0!! " << std::endl;
                std::getline(std::cin, grasped_object_id_str);
            }
            grasped_object_id = std::stoi(grasped_object_id_str);
        }
        else{ // if we have target object transform, then select the target object based on it.
            grasped_object_id = 0;
            float min_distance = 1000000;
            for(int i = 0; i < obstacle_srv.response.segmented_objects.objects.size(); i++)
            {
                float diffDistance = (target_object_transform.getOrigin() - tf::Vector3(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.x, obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.y, obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.z)).length();
                if(diffDistance < min_distance)
                {
                    min_distance = diffDistance;
                    grasped_object_id = i;
                }
            }
            if(min_distance == 1000000)
            {
                ROS_ERROR("No object found on the table");
                return 0;
            }
        }

        std::cout << "grasped object id: " << grasped_object_id << std::endl;
        // is_target[grasped_object_id] = true;

        // need to analyze which obstacle should be moved away before grasping the targect object.
        int last_front_obstacle_id = grasped_object_id;
        int front_obstacle_id = last_front_obstacle_id;
        // if(use_regrasp) // if regrasp is used, then we need to move the obstacle in front of the target object.
        if(false)// need to change back for remove occluding object.
        {
            do{
                front_obstacle_id = last_front_obstacle_id;
                float cx = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.position.x;
                float cy = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.position.y;
                float c_norm = sqrt(cx*cx + cy*cy);
                float c_size = (obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x + obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y ) / 4.0;
                for(int i = 0; i < obstacle_srv.response.segmented_objects.objects.size(); i++)
                {
                    if(i == front_obstacle_id)
                        continue;
                    float ox = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.x;
                    float oy = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.y;
                    float o_norm = sqrt(ox*ox + oy*oy);

                    float distancetoline = o_norm * sqrt(1.0 - pow((cx * ox + cy * oy) / (o_norm * c_norm),2));
                    if(c_norm > o_norm && distancetoline < c_size)
                    {
                        last_front_obstacle_id = i;
                        continue;
                    }
                }
            }while(front_obstacle_id != last_front_obstacle_id);        
        }
        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        std::cout << "the object should be grasped now is: " << front_obstacle_id << std::endl;

        // init all information of the current grasped object. vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        // get the target object transform
        target_object_transform.setOrigin(tf::Vector3(obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.position.x, 
                                                        obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.position.y, 
                                                        obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.position.z));
        target_object_transform.setRotation(tf::Quaternion(obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.orientation.x, 
                                                            obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.orientation.y, 
                                                            obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.orientation.z, 
                                                            obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.pose.pose.orientation.w));

        // initialize the target object as the attached object.
        shapes::Shape* target_object_shape = new shapes::Box(obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x,
                                                            obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y,
                                                            obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.z);
        std::vector<shapes::ShapeConstPtr> target_object_shapes;
        target_object_shapes.push_back(shapes::ShapeConstPtr(target_object_shape));
        EigenSTL::vector_Isometry3d shape_poses{Eigen::Isometry3d::Identity()};

        // init the collision object for target object
        moveit_msgs::CollisionObject collision_object_target;
        collision_object_target.header.frame_id = move_group.getPlanningFrame();
        collision_object_target.id =  "target_object";
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        // need to predict both grasp poses and object com, the visualize them vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        // we need to extract the grasped object point cloud with full point cloud.
        ros_tensorflow_msgs::Predict grasp_prediction_srv;
        grasp_prediction_srv.request.full_point_cloud = table_srv.response.full_point_cloud;
        grasp_prediction_srv.request.segmented_point_cloud = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].point_cloud;
        grasp_prediction_srv.request.camera_stamped_transform = camera_stamped_transform;

        // prepare croppped table pcs for CoM prediction
        sensor_msgs::PointCloud2 cropped_table_point_cloud;
        generatePointCloudOfPlane(target_object_transform, 
                            obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y * 1.7,
                            obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.z * 1.7,
                            obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x,
                            0.005,
                            cropped_table_point_cloud);

        ros_tensorflow_msgs::ComPredict com_predict_srv;
        com_predict_srv.request.table_point_cloud = cropped_table_point_cloud;
        com_predict_srv.request.segmented_point_cloud = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].point_cloud;
        com_predict_srv.request.camera_stamped_transform = camera_stamped_transform;

        if (not grasp_prediction_client.call(grasp_prediction_srv))
        {
            ROS_ERROR("Failed to call service grasp prediction");
            return 1;
        }
        
        if (not com_prediction_client.call(com_predict_srv))
        {
            ROS_ERROR("Failed to call service center of mass prediction");
            return 1;
        }

        tf::Vector3 target_com;
        target_com.setX(com_predict_srv.response.com.x);
        target_com.setY(com_predict_srv.response.com.y);
        target_com.setZ(com_predict_srv.response.com.z);

        tf::Vector3 target_com_in_object_frame = target_object_transform.inverse() * target_com;
        //*************************************************************************************//

        // get object pose from tf TODO: delete if COMpredict is working
        // std::vector<std::string> object_names = {"can", "book", "bottle", "hammer"};
        // for(std::string ob: object_names)
        // {   
        //     try{
        //         tf::StampedTransform object_transform_temp;
        //         ros::Time now = ros::Time::now();
        //         listener.waitForTransform("/base_link", "/" + ob, ros::Time(0), ros::Duration(1.0));
        //         listener.lookupTransform("/base_link", "/" + ob, ros::Time(0), object_transform_temp);

        //         tf::Vector3 distanceToCom = target_object_transform.inverse() * object_transform_temp.getOrigin();

        //         if(abs(distanceToCom.x()) < obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x / 2.0 &&
        //         abs(distanceToCom.y()) < obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y / 2.0 &&
        //         abs(distanceToCom.z()) < obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.z / 2.0)
        //         {
        //             target_com.setX(object_transform_temp.getOrigin().x());
        //             target_com.setY(object_transform_temp.getOrigin().y());
        //             target_com.setZ(object_transform_temp.getOrigin().z());
        //             break;
        //         }
        //     }
        //     catch(tf::TransformException ex){
        //         ROS_ERROR("%s", ex.what());
        //         ros::Duration(1.0).sleep();
        //     }
        // }

        // manipulation_test::VisualizeCom com_visualize_srv;
        // // com_visualize_srv.request.com = com_predict_srv.response.com;
        // std::cout << "predicted com: " << target_com.x() << ", " << target_com.y() << ", " << target_com.z() << std::endl;
        // geometry_msgs::Point com_point;
        // com_point.x = target_com.x();
        // com_point.y = target_com.y();
        // com_point.z = target_com.z();
        // com_visualize_srv.request.com = com_point;
        // com_visualize_srv.request.segmented_point_cloud = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].point_cloud;
        // std::cout << "segmented point cloud size: " << obstacle_srv.response.segmented_objects.objects[front_obstacle_id].point_cloud.data.size() << std::endl;
        // com_visualize_srv.request.table_point_cloud = cropped_table_point_cloud;
        // std::cout << "table point cloud size: " << cropped_table_point_cloud.data.size() << std::endl;

        // com_predict_visualizer.call(com_visualize_srv);

        /************************************************************************************/

        // init the variables for grasping
        std::vector<tf::Transform> grasp_transforms_before_clustering;
        std::vector<float> grasp_jawwidths_before_clustering;
        std::vector<int> grasp_types_before_clustering;

        // add the lifting grasp poses first
        for(int i = 0; i < grasp_prediction_srv.response.predicted_grasp_poses.size(); i++){
            if(grasp_prediction_srv.response.scores[i] < 0.3)
                continue;
            tf::Stamped<tf::Transform> predicted_grasp_transform;
            tf::poseStampedMsgToTF(grasp_prediction_srv.response.predicted_grasp_poses[i], predicted_grasp_transform);
            if(lift_torque_test(target_com, obj_mass, predicted_grasp_transform))
            {
                // if we just want to use contact grasp net alone, we can to here directly
                grasp_transforms_before_clustering.push_back(target_object_transform.inverse() * predicted_grasp_transform);
                grasp_jawwidths_before_clustering.push_back(0.08);
                grasp_types_before_clustering.push_back(0);
            }
        }

        if(grasp_transforms_before_clustering.size() == 0)
        {
            if(num_of_trials == 0)
            {
                ROS_ERROR("No lifting grasp found by Contact Grasp Net, breaking");
                return 1;
            }
            else{
                num_of_trials--;
                retry = true;
                hasTargetObject = true;
                ROS_INFO("No lifting grasp found by Contact Grasp Net, trying again");
                continue;
            }
        }

        // then add sliding grasp poses
        for(int i = 0; i < grasp_prediction_srv.response.predicted_grasp_poses.size(); i++){
            if(grasp_prediction_srv.response.scores[i] < 0.3)
                continue;
            tf::Stamped<tf::Transform> predicted_grasp_transform;
            tf::poseStampedMsgToTF(grasp_prediction_srv.response.predicted_grasp_poses[i], predicted_grasp_transform);
            if(!lift_torque_test(target_com, obj_mass, predicted_grasp_transform))
            {
                grasp_transforms_before_clustering.push_back(target_object_transform.inverse() * predicted_grasp_transform);
                grasp_jawwidths_before_clustering.push_back(0.08);
                grasp_types_before_clustering.push_back(1);
            }
        }

        // grasp pose clustering
        std::vector<tf::Transform> grasp_transforms;
        std::vector<float> grasp_jawwidths;
        std::vector<int> grasp_types;

        for(int i = 0; i < grasp_transforms_before_clustering.size(); i++){
            bool is_clustered = false;
            tf::Transform grasp_point_transform = grasp_transforms_before_clustering[i] * tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0, 0));
            for(int j = 0; j < grasp_transforms.size(); j++){
                tf::Transform grasp_point_transform_clustered = grasp_transforms[j] * tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0, 0));
                if(grasp_point_transform.getRotation().angle(grasp_point_transform_clustered.getRotation()) < 0.6){
                    if(grasp_point_transform.getOrigin().distance(grasp_point_transform_clustered.getOrigin()) < 0.06){
                        is_clustered = true;
                        break;
                    }
                }
            }
            if(!is_clustered){
                grasp_transforms.push_back(grasp_transforms_before_clustering[i]);
                grasp_jawwidths.push_back(grasp_jawwidths_before_clustering[i]);
                grasp_types.push_back(grasp_types_before_clustering[i]);
            }
        }

        // grasp pose visualization
        manipulation_test::VisualizeRegrasp grasp_poses_visualize_srv;
        for(int i = 0; i < grasp_transforms.size(); i++){
            tf::Transform grasp_transform_in_world = target_object_transform * grasp_transforms[i];
            geometry_msgs::PoseStamped grasp_pose_msg;
            grasp_pose_msg.header.frame_id = "base_link";
            grasp_pose_msg.pose.position.x = grasp_transform_in_world.getOrigin().x();
            grasp_pose_msg.pose.position.y = grasp_transform_in_world.getOrigin().y();
            grasp_pose_msg.pose.position.z = grasp_transform_in_world.getOrigin().z();

            grasp_pose_msg.pose.orientation.x = grasp_transform_in_world.getRotation().x();
            grasp_pose_msg.pose.orientation.y = grasp_transform_in_world.getRotation().y();
            grasp_pose_msg.pose.orientation.z = grasp_transform_in_world.getRotation().z();
            grasp_pose_msg.pose.orientation.w = grasp_transform_in_world.getRotation().w();

            grasp_poses_visualize_srv.request.grasp_poses.push_back(grasp_pose_msg);
            grasp_poses_visualize_srv.request.grasp_jawwidths.push_back(grasp_jawwidths[i]);
            grasp_poses_visualize_srv.request.grasp_types.push_back(grasp_types[i]);
        }
        if (!grasp_poses_visualizer.call(grasp_poses_visualize_srv))
        {
            ROS_ERROR("Failed to call service visualize_regrasp");
            return 1;
        }

        // wait for user input
        std::cout << "Do you like the grasps? () " << std::endl;
        std::string accept_grasp_str;
        std::getline(std::cin, accept_grasp_str);

        while(accept_grasp_str != "y" && accept_grasp_str != "n")
        {
            std::cout << "You should enter y or n!! " << std::endl;
            std::getline(std::cin, accept_grasp_str);
        }
        
        if(accept_grasp_str == "n")
        {
            // clear planning scene
            planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
            return 0;
        }

        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        // initialize the collision environment vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryf>;
        using CollisionObjectPtr_t = std::shared_ptr<fcl::CollisionObjectf>;
        std::vector<CollisionObjectPtr_t> collision_objects;
        CollisionObjectPtr_t target_collision_object;

        for(int i = 0; i < obstacle_srv.response.segmented_objects.objects.size(); i++)
        {
            // obstacle_visualize_srv.request.is_target[i] = is_target[i];
            // if(is_target[i] == false)
            if(i != front_obstacle_id)
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

                // add the obstacle into the planning scene
                moveit_msgs::CollisionObject collision_object_obstacle;
                collision_object_obstacle.header.frame_id = move_group.getPlanningFrame();

                collision_object_obstacle.id = "obstacle_" + std::to_string(i);

                shape_msgs::SolidPrimitive obstacle_primitive;
                obstacle_primitive.type = obstacle_primitive.BOX;
                obstacle_primitive.dimensions.resize(3);
                obstacle_primitive.dimensions[0] = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.x;
                obstacle_primitive.dimensions[1] = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.y;
                obstacle_primitive.dimensions[2] = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.z;

                geometry_msgs::Pose obstacle_pose;
                obstacle_pose.position.x = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.x;
                obstacle_pose.position.y = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.y;
                obstacle_pose.position.z = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.position.z;

                obstacle_pose.orientation.x = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.orientation.x;
                obstacle_pose.orientation.y = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.orientation.y;
                obstacle_pose.orientation.z = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.orientation.z;
                obstacle_pose.orientation.w = obstacle_srv.response.segmented_objects.objects[i].bounding_volume.pose.pose.orientation.w;


                collision_object_obstacle.primitives.push_back(obstacle_primitive);
                collision_object_obstacle.pose = obstacle_pose;

                collision_object_obstacle.operation = collision_object_obstacle.ADD;
                planning_scene_interface.applyCollisionObject(collision_object_obstacle);
            }
            else
            {
                obstacle_visualize_srv.request.is_target[i] = true;
                // create the collision object with only geometry
                CollisionGeometryPtr_t box_geom = std::make_shared<fcl::Boxf>(obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.x, 
                                                                            obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.y, 
                                                                            obstacle_srv.response.segmented_objects.objects[i].bounding_volume.dimensions.z);
                target_collision_object = std::make_shared<fcl::CollisionObjectf>(box_geom);
            }
        }

        // need to re-visualize the obstacles on the table by different color for the target object
        if (not obstacle_visualizer.call(obstacle_visualize_srv))
        {
            ROS_ERROR("Failed to call service visualize_obstacle");
            return 1;
        }
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        std::vector<std::pair<std::string, robot_trajectory::RobotTrajectory>> robot_action_trajectory_execution_list;
        bool need_to_reanalyze = true;
        tf::Transform target_object_transform_during_regrasp;

        if(!use_regrasp) // if we don't want to use regrasp, we plan the motion grasp the object directly.
        {
            bool find_solution = false;
            // no need to re analyze the object if we don't use regrasp
            re_analyze = false;
            collision_object_table.operation = collision_object_table.ADD;
            planning_scene_interface.applyCollisionObject(collision_object_table);
            move_group.setPlanningTime(1.5);
            for(int g = 0; g < grasp_transforms.size(); g++)
            {
                robot_action_trajectory_execution_list.clear();

                // calculate the pre-grasp, grasp and lift-grasp poses
                tf::Transform pre_grasp_pose_in_world_frame = target_object_transform * grasp_transforms[g] * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.1, 0, 0));
                tf::Transform grasp_pose_in_world_frame = target_object_transform * grasp_transforms[g];
                tf::Transform lift_grasp_pose_in_world_frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0, 0, 0.1)) * target_object_transform * grasp_transforms[g];
                geometry_msgs::Pose pre_grasp_pose;
                geometry_msgs::Pose grasp_pose;
                geometry_msgs::Pose lift_grasp_pose;
                transformTFToGeoPose(pre_grasp_pose_in_world_frame, pre_grasp_pose);
                transformTFToGeoPose(grasp_pose_in_world_frame, grasp_pose);
                transformTFToGeoPose(lift_grasp_pose_in_world_frame, lift_grasp_pose);

                // refresh the current state
                moveit::core::RobotState current_state = *(move_group.getCurrentState());

                // plan to pre-grasp pose
                move_group.setStartState(current_state);
                move_group.setPoseTarget(pre_grasp_pose);
                moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
                if(move_group.plan(pregrasp_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    continue;
                robot_trajectory::RobotTrajectory pregrasp_trajectory = 
                                robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, pregrasp_plan.trajectory_);                
                robot_action_trajectory_execution_list.push_back(std::make_pair("arm", pregrasp_trajectory));
                current_state = pregrasp_trajectory.getLastWayPoint();

                // open the gripper
                end_effector_move_group.setStartState(current_state);
                for(int i = 0; i < finger_joint_names.size(); i++)
                    end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.04);
                moveit::planning_interface::MoveGroupInterface::Plan open_gripper_plan;
                if(end_effector_move_group.plan(open_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    continue;
                robot_trajectory::RobotTrajectory open_gripper_trajectory = 
                                robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, open_gripper_plan.trajectory_);
                robot_action_trajectory_execution_list.push_back(std::make_pair("open", open_gripper_trajectory));
                current_state = open_gripper_trajectory.getLastWayPoint();

                // approach the object
                move_group.setStartState(current_state);
                moveit_msgs::RobotTrajectory approach_trajectory_msg;    
                if(move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{grasp_pose}, 0.003, 5.0, approach_trajectory_msg, true) < 0.95)
                    continue;
                robot_trajectory::RobotTrajectory approach_trajectory = 
                                robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, approach_trajectory_msg);
                robot_action_trajectory_execution_list.push_back(std::make_pair("arm", approach_trajectory));
                current_state = approach_trajectory.getLastWayPoint();

                // close the gripper
                end_effector_move_group.setStartState(current_state);
                for(int i = 0; i < finger_joint_names.size(); i++)
                    end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.0);
                moveit::planning_interface::MoveGroupInterface::Plan close_gripper_plan;
                if(end_effector_move_group.plan(close_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    continue;
                robot_trajectory::RobotTrajectory close_gripper_trajectory = 
                                robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, close_gripper_plan.trajectory_);
                robot_action_trajectory_execution_list.push_back(std::make_pair("close", close_gripper_trajectory));
                current_state = close_gripper_trajectory.getLastWayPoint();

                // plan to lift the object
                move_group.setStartState(current_state);
                moveit_msgs::RobotTrajectory lift_trajectory_msg;
                if(move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{lift_grasp_pose}, 0.005, 5.0, lift_trajectory_msg, true) < 0.95)
                    continue;
                robot_trajectory::RobotTrajectory lifting_trajectory = 
                                robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, lift_trajectory_msg);
                robot_action_trajectory_execution_list.push_back(std::make_pair("lift", lifting_trajectory));
                current_state = lifting_trajectory.getLastWayPoint();

                // need to move to the position for placing
                geometry_msgs::Pose current_in_hand_pose_for_placing;
                transformTFToGeoPose(grasp_transforms[g].inverse(), current_in_hand_pose_for_placing);

                // need to place the object with constrained based rrt planning.
                move_group.setPlannerId("CBIRRTConfigDefault");
                move_group.setPlanningTime(40.0);
                move_group.setStartState(current_state);

                // set the placing constraints
                moveit_msgs::Constraints placing_constraints;
                placing_constraints.name = "use_equality_constraints";
                placing_constraints.in_hand_pose = current_in_hand_pose_for_placing;
                
                // define the orientation constraint on the object
                moveit_msgs::OrientationConstraint orientation_constraint;
                orientation_constraint.parameterization = moveit_msgs::OrientationConstraint::ROTATION_VECTOR;
                orientation_constraint.header.frame_id = "base_link";
                orientation_constraint.header.stamp = ros::Time(0);
                orientation_constraint.link_name = "wrist_roll_link";
                geometry_msgs::Quaternion constrained_quaternion;
                constrained_quaternion.x = target_object_transform.getRotation().x();
                constrained_quaternion.y = target_object_transform.getRotation().y();
                constrained_quaternion.z = target_object_transform.getRotation().z();
                constrained_quaternion.w = target_object_transform.getRotation().w();
                orientation_constraint.orientation = constrained_quaternion;
                orientation_constraint.weight = 1.0;
                orientation_constraint.absolute_x_axis_tolerance = 2 * 3.1415;
                orientation_constraint.absolute_y_axis_tolerance = 0.1;
                orientation_constraint.absolute_z_axis_tolerance = 0.1;
                placing_constraints.orientation_constraints.push_back(orientation_constraint);

                // calculate current target object pose.
                tf::Transform placing_transform;
                Eigen::Isometry3d placing_eigen_transform;
                GeoPoseTotransformTF(current_in_hand_pose_for_placing, placing_transform);
                tf::transformTFToEigen(placing_transform, placing_eigen_transform);
                current_state.attachBody("target_object", placing_eigen_transform, target_object_shapes, shape_poses, std::vector<std::string>{"l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"}, "wrist_roll_link");

                move_group.setPathConstraints(placing_constraints);
                move_group.setInHandPose(current_in_hand_pose_for_placing);

                move_group.setPositionTarget(0.34, -0.48, 0.93);
                move_group.setCleanPlanningContextFlag(true);
                moveit::planning_interface::MoveGroupInterface::Plan placing_plan;
                bool success = (move_group.plan(placing_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                current_state.clearAttachedBody("target_object");

                move_group.clearPathConstraints();
                move_group.clearInHandPose();
                move_group.clearAction();
                // move_group.clearExperience();
                move_group.clearPoseTargets();
                move_group.clearInHandPose();

                if(!success)
                {
                    std::cout << "placing plan failed" << std::endl;
                    continue;
                }
                std::cout << "placing plan success" << std::endl;
                robot_trajectory::RobotTrajectory placing_trajectory = 
                                robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, placing_plan.trajectory_);

                robot_action_trajectory_execution_list.push_back(std::make_pair("arm", placing_trajectory));
                current_state = placing_trajectory.getLastWayPoint();

                // get the end effector pose of current state
                tf::Transform current_hand_transform;
                tf::transformEigenToTF(current_state.getGlobalLinkTransform("wrist_roll_link"), current_hand_transform);
                geometry_msgs::Pose place_down_grasp_pose;
                transformTFToGeoPose(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0, 0, -0.1)) * current_hand_transform, place_down_grasp_pose);

                // need to place down the object
                move_group.setStartState(current_state);
                moveit_msgs::RobotTrajectory place_down_trajectory_msg;    
                if(move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{place_down_grasp_pose}, 0.005, 5.0, place_down_trajectory_msg, true) < 0.95)
                    continue;
                robot_trajectory::RobotTrajectory place_down_trajectory = 
                                robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, place_down_trajectory_msg);
                robot_action_trajectory_execution_list.push_back(std::make_pair("arm", place_down_trajectory));
                current_state = place_down_trajectory.getLastWayPoint();

                // open the gripper
                end_effector_move_group.setStartState(current_state);
                for(int i = 0; i < finger_joint_names.size(); i++)
                    end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.04);
                if(end_effector_move_group.plan(open_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    continue;
                open_gripper_trajectory = 
                                robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, open_gripper_plan.trajectory_);
                robot_action_trajectory_execution_list.push_back(std::make_pair("open", open_gripper_trajectory));
                current_state = open_gripper_trajectory.getLastWayPoint();

                find_solution = true;

                break;
            }
            collision_object_table.operation = collision_object_table.REMOVE;
            planning_scene_interface.applyCollisionObject(collision_object_table);

            if(!find_solution)
            {
                std::cout << "can't find solution without regrasping" << std::endl;
                return 0;
            }
        }
        else{

            // initialize varaibles to find all feasible re-grasp poses and intermedaite placements
            // the following information will be used to initialize the task planner
            std::vector<tf::Transform> feasible_intermediate_placement_transforms;
            std::vector<std::vector<tf::Transform>> feasible_regrasp_transforms;
            std::vector<std::vector<int>> regrasp_ids;
            std::vector<std::vector<int>> regrasp_types;
            std::vector<std::vector<std::vector<moveit_msgs::RobotTrajectory>>> approach_motions;
            std::vector<std::vector<std::vector<moveit_msgs::RobotTrajectory>>> lifting_motions;

            // try to add the current object pose as one of the intermediate placement
            std::vector<tf::Transform> feasible_regrasp_transforms_for_init;  // feasible re-grasp poses in this intermediate placement
            std::vector<int> regrasp_ids_for_init; // the id of re-grasp poses in this intermediate placement
            std::vector<int> regrasp_types_for_init; // the type of re-grasp poses in this intermediate placement
            std::vector<std::vector<moveit_msgs::RobotTrajectory>> approach_motions_for_init; // the lifting motions of feasible re-grasp poses in this intermediate placement
            std::vector<std::vector<moveit_msgs::RobotTrajectory>> lifting_motions_for_init; // the lifting motions of feasible re-grasp poses in this intermediate placement

            moveit::core::RobotState robotStatetemp = *(move_group.getCurrentState());

            int numberOfGraspPosesCanLiftDirectly = 0;

            // in the current target object transform, check if there is any feasible re-grasp pose and liftup grasp pose.
            for(int g = 0; g < grasp_transforms.size(); g++)
            {
                tf::Transform pre_grasp_pose_in_world_frame = target_object_transform * grasp_transforms[g] * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.1, 0, 0));
                tf::Transform lift_grasp_pose_in_world_frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0, 0, 0.1)) * target_object_transform * grasp_transforms[g];
                geometry_msgs::Pose pre_grasp_pose;
                geometry_msgs::Pose lift_grasp_pose;
                transformTFToGeoPose(pre_grasp_pose_in_world_frame, pre_grasp_pose);
                transformTFToGeoPose(lift_grasp_pose_in_world_frame, lift_grasp_pose);

                std::vector<moveit_msgs::RobotTrajectory> current_approach_motions; // the approach motions of current grasp pose
                std::vector<moveit_msgs::RobotTrajectory> current_lifting_motions; // the lifting motions of current grasp pose
                // desired end-effector pose
                KDL::Frame end_effector_pose;
                transformTFToKDL(torso_transform.inverse() * target_object_transform * grasp_transforms[g], end_effector_pose);

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
                        setRobotState(robotStatetemp, random_joint_array, joint_names);

                        // check whether the start state is valid.
                        moveit_msgs::GetStateValidity::Request validity_request;
                        moveit_msgs::GetStateValidity::Response validity_response;
                        robot_state::robotStateToRobotStateMsg(robotStatetemp, validity_request.robot_state);
                        validity_request.group_name = PLANNING_GROUP;

                        validity_client.call(validity_request, validity_response);
                        if (validity_response.valid)
                        {
                            // check the cartesian path
                            move_group.setStartState(robotStatetemp);
                            moveit_msgs::RobotTrajectory approach_trajectory_msg;
                            moveit_msgs::RobotTrajectory lifting_trajectory_msg;

                            double pre_grasp_fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{pre_grasp_pose}, 0.003, 5.0, approach_trajectory_msg, false);
                            double lift_grasp_fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{lift_grasp_pose}, 0.005, 5.0, lifting_trajectory_msg, false);

                            if(pre_grasp_fraction >= 0.95 && lift_grasp_fraction >= 0.95)
                            {
                                // check the validity of the end point of the path
                                robot_state::RobotState end_state = robotStatetemp;
                                end_state.setVariablePositions(joint_names, approach_trajectory_msg.joint_trajectory.points.back().positions);
                                robot_state::robotStateToRobotStateMsg(end_state, validity_request.robot_state);

                                validity_client.call(validity_request, validity_response);

                                bool approach_valid = validity_response.valid;

                                end_state.setVariablePositions(joint_names, lifting_trajectory_msg.joint_trajectory.points.back().positions);
                                robot_state::robotStateToRobotStateMsg(end_state, validity_request.robot_state);

                                validity_client.call(validity_request, validity_response);

                                bool lifting_valid = validity_response.valid;

                                if (approach_valid && lifting_valid)
                                {
                                    current_lifting_motions.push_back(lifting_trajectory_msg);
                                    current_approach_motions.push_back(approach_trajectory_msg);
                                }
                            }
                        }
                        move_group.clearPoseTargets();
                    }

                    if(current_approach_motions.size() > 2) // no need to find more lifting motions
                        break;
                }

                if(current_approach_motions.size() > 0)
                { // this grasp pose can be used to lift the object.
                    approach_motions_for_init.push_back(current_approach_motions);
                    lifting_motions_for_init.push_back(current_lifting_motions);
                    feasible_regrasp_transforms_for_init.push_back(pre_grasp_pose_in_world_frame);
                    regrasp_ids_for_init.push_back(g);
                    regrasp_types_for_init.push_back(grasp_types[g]);
                }
            }

            // check the number of grasp pose can be used to lift up the object.
            for(int grasp_type_n: regrasp_types_for_init)
                if(grasp_type_n == 0)
                    numberOfGraspPosesCanLiftDirectly++;
                
            if(numberOfGraspPosesCanLiftDirectly == 0)
            {
                // convert the table point cloud to pcl point cloud
                pcl::PCLPointCloud2 pcl_pc2;
                pcl_conversions::toPCL(table_srv.response.point_cloud, pcl_pc2);
                pcl::PointCloud<pcl::PointXYZ> table_point_cloud;
                pcl::fromPCLPointCloud2(pcl_pc2,table_point_cloud);
                
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

                    // need to ensure the re-grasping pose is in the camera view
                    tf::Vector3 object_x_3d = camera_transform.inverse() * random_target_object_transform * tf::Vector3(obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x * 0.8,0,0);
                    tf::Vector3 object_y_3d = camera_transform.inverse() * random_target_object_transform * tf::Vector3(0,obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y * 0.8,0);
                    tf::Vector3 object_z_3d = camera_transform.inverse() * random_target_object_transform * tf::Vector3(0,0,obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.z * 0.8);
                    tf::Vector3 object_x_3d_neg = camera_transform.inverse() * random_target_object_transform * tf::Vector3(-obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x * 0.8,0,0);
                    tf::Vector3 object_y_3d_neg = camera_transform.inverse() * random_target_object_transform * tf::Vector3(0,-obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y * 0.8,0);
                    tf::Vector3 object_z_3d_neg = camera_transform.inverse() * random_target_object_transform * tf::Vector3(0,0,-obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.z * 0.8);

                    // for object_x_3d
                    if(!is_in_view(object_x_3d, camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5], camera_info.width, camera_info.height))
                        continue;

                    // for object_y_3d
                    if(!is_in_view(object_y_3d, camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5], camera_info.width, camera_info.height))
                        continue;

                    // for object_z_3d
                    if(!is_in_view(object_z_3d, camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5], camera_info.width, camera_info.height))
                        continue;

                    // for object_x_3d_neg
                    if(!is_in_view(object_x_3d_neg, camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5], camera_info.width, camera_info.height))
                        continue;

                    // for object_y_3d_neg
                    if(!is_in_view(object_y_3d_neg, camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5], camera_info.width, camera_info.height))
                        continue;

                    // for object_z_3d_neg
                    if(!is_in_view(object_z_3d_neg, camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5], camera_info.width, camera_info.height))
                        continue;

                    // the object's com should be on the table during regrasping.
                    tf::Vector3 random_object_com = random_target_object_transform_in_table_frame * target_com_in_object_frame;
                    if(random_object_com.x() <  - table_srv.response.depth / 2.0 || random_object_com.x() > table_srv.response.depth / 2.0)
                        continue;

                    if(random_object_com.y() <  - table_srv.response.width / 2.0 || random_object_com.y() > table_srv.response.width / 2.0)
                        continue;

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
                        continue;
                    
                    random_target_object_transforms.push_back(random_target_object_transform);
                    if(random_target_object_transforms.size() >= 5)
                        break;
                }
                
                // find all feasible re-grasp poses in those intermediate placements
                for(tf::Transform intermedaite_placement_transform: random_target_object_transforms)
                {
                    int number_of_feable_grasp_poses = 0;
                    std::vector<bool> is_feasible_grasp_poses(grasp_transforms.size(), false);
                    for(int i = 0; i < grasp_transforms.size(); i++)
                    {
                        // check whether the grasp pose is feasible
                        KDL::Frame end_effector_pose;
                        KDL::JntArray result = nominal;
                        transformTFToKDL(torso_transform.inverse() * intermedaite_placement_transform * grasp_transforms[i], end_effector_pose);
                        if(kdl_solver.CartToJnt(result, end_effector_pose, result) >= 0)
                        {
                            is_feasible_grasp_poses[i] = true;
                            number_of_feable_grasp_poses++;
                        }
                    }

                    if(number_of_feable_grasp_poses < 2) // there are less than two feasible grasp poses in this intermediate placement for re-grasping.
                        continue;

                    // find all feasible re-grasp poses with its lifting motion in this intermediate placement
                    std::vector<tf::Transform> feasible_regrasp_transforms_temp;  // feasible re-grasp poses in this intermediate placement
                    std::vector<int> regrasp_ids_temp; // the id of re-grasp poses in this intermediate placement
                    std::vector<int> regrasp_types_temp; // the type of re-grasp poses in this intermediate placement
                    std::vector<std::vector<moveit_msgs::RobotTrajectory>> approach_motions_temp; // the lifting motions of feasible re-grasp poses in this intermediate placement
                    std::vector<std::vector<moveit_msgs::RobotTrajectory>> lifting_motions_temp;
                    
                    // search all state in this intermediate placement
                    for(int g = 0; g < grasp_transforms.size(); g++)
                    {
                        if(!is_feasible_grasp_poses[g]) // this grasp pose is not feasible so skip it.
                            continue;

                        // find pre-grasp pose and transform of current grasp pose.
                        tf::Transform pre_grasp_pose_in_world_frame = intermedaite_placement_transform * grasp_transforms[g] * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.1, 0, 0));
                        tf::Transform lift_grasp_pose_in_world_frame = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0, 0, 0.1)) * intermedaite_placement_transform * grasp_transforms[g];
                        geometry_msgs::Pose pre_grasp_pose;
                        geometry_msgs::Pose lift_grasp_pose;
                        transformTFToGeoPose(pre_grasp_pose_in_world_frame, pre_grasp_pose);
                        transformTFToGeoPose(lift_grasp_pose_in_world_frame, lift_grasp_pose);

                        std::vector<moveit_msgs::RobotTrajectory> current_approach_motions; // the lifting motions of current grasp pose
                        std::vector<moveit_msgs::RobotTrajectory> current_lifting_motions;
                        // desired end-effector pose
                        KDL::Frame end_effector_pose;
                        transformTFToKDL(torso_transform.inverse() * intermedaite_placement_transform * grasp_transforms[g], end_effector_pose);

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
                                setRobotState(robotStatetemp, random_joint_array, joint_names);

                                // check whether the start state is valid.
                                moveit_msgs::GetStateValidity::Request validity_request;
                                moveit_msgs::GetStateValidity::Response validity_response;
                                robot_state::robotStateToRobotStateMsg(robotStatetemp, validity_request.robot_state);
                                validity_request.group_name = PLANNING_GROUP;

                                validity_client.call(validity_request, validity_response);
                                if (validity_response.valid)
                                {
                                    // check the cartesian path
                                    move_group.setStartState(robotStatetemp);
                                    moveit_msgs::RobotTrajectory approach_trajectory_msg;
                                    moveit_msgs::RobotTrajectory lifting_trajectory_msg;

                                    double pre_grasp_fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{pre_grasp_pose}, 0.003, 5.0, approach_trajectory_msg, false);
                                    double lift_grasp_fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{lift_grasp_pose}, 0.005, 5.0, lifting_trajectory_msg, false);
                
                                    if(pre_grasp_fraction >= 0.95 && lift_grasp_fraction >= 0.95)
                                    {
                                        // check the validity of the end point of the path
                                        robot_state::RobotState end_state = robotStatetemp;
                                        end_state.setVariablePositions(joint_names, approach_trajectory_msg.joint_trajectory.points.back().positions);
                                        robot_state::robotStateToRobotStateMsg(end_state, validity_request.robot_state);

                                        validity_client.call(validity_request, validity_response);

                                        bool approach_valid = validity_response.valid;

                                        end_state.setVariablePositions(joint_names, lifting_trajectory_msg.joint_trajectory.points.back().positions);
                                        robot_state::robotStateToRobotStateMsg(end_state, validity_request.robot_state);

                                        validity_client.call(validity_request, validity_response);

                                        bool lifting_valid = validity_response.valid;

                                        if (approach_valid && lifting_valid)
                                        {
                                            current_lifting_motions.push_back(lifting_trajectory_msg);
                                            current_approach_motions.push_back(approach_trajectory_msg);
                                        }
                                    }
                                }
                                    
                                move_group.clearPoseTargets();
                            }

                            if(current_approach_motions.size() > 2) // no need to find more lifting motions
                                break;
                        }

                        if(current_approach_motions.size() > 0) // has a way to lift up the object with current grasp pose 
                        {
                            approach_motions_temp.push_back(current_approach_motions);
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
                        approach_motions.push_back(approach_motions_temp);
                        lifting_motions.push_back(lifting_motions_temp);
                    }
                }
            }
            else
            {
                // mean the target object can be grasped directly, no need to re-analyze
                re_analyze = false;
            }

            if(feasible_regrasp_transforms_for_init.size() > 0)
            {
                feasible_intermediate_placement_transforms.push_back(target_object_transform);
                feasible_regrasp_transforms.push_back(feasible_regrasp_transforms_for_init);
                regrasp_ids.push_back(regrasp_ids_for_init);
                regrasp_types.push_back(regrasp_types_for_init);
                approach_motions.push_back(approach_motions_for_init);
                lifting_motions.push_back(lifting_motions_for_init);
            }
            else
            {
                if(num_of_trials == 0){
                    ROS_INFO("grasps from Contact GraspNet are not approachable.");
                    return 0;
                }
                else{
                    num_of_trials--;
                    retry = true;
                    hasTargetObject = true;
                    ROS_INFO("grasps from Contact GraspNet are not approachable, try again");
                    continue;
                }
            }
        
            moveit::core::RobotState current_state = *(move_group.getCurrentState());

            // count the number of grasps to lift the object.
            int num_of_lifting_grasps = 0;

            //init the task planner.
            TaskPlanner task_planner(std::vector<long unsigned int>{feasible_intermediate_placement_transforms.size(), 0}, std::vector<long unsigned int>{grasp_transforms.size(), grasp_transforms.size()});
            // in the second foliation, there is no intermediate placement, so we set it to be zero.

            // add all action motions into the task planner.
            // for(int f = 0; f < 1; f++)
            // {
            //     for(int p = 0; p < feasible_intermediate_placement_transforms.size(); p++)
            //     {
            //         bool is_init_action = false;
            //         // the last placmeent should always be the current object pose.
            //         if(p == feasible_intermediate_placement_transforms.size()-1)
            //             is_init_action = true;

            //         for(int g = 0; g < regrasp_ids[p].size(); g++)
            //         {
            //             if(regrasp_types[p][g] == 0) // this grasp pose can be used to lift the object.
            //             {
            //                 num_of_lifting_grasps++;
            //                 for(int k = 0; k < lifting_motions[p][g].size(); k++)
            //                 {
            //                     task_planner.addActionBetweenFoliations(lifting_motions[p][g][k], f, regrasp_ids[p][g], f+1, regrasp_ids[p][g], 1.0);
            //                 }
            //             }
            //             for(int k = 0; k < approach_motions[p][g].size(); k++)
            //             {
            //                 task_planner.addActionBetweenManifolds(approach_motions[p][g][k], p, regrasp_ids[p][g], f, is_init_action, -0.005);
            //             }
            //         }
            //     }
            // }
            for(int p = 0; p < feasible_intermediate_placement_transforms.size(); p++)
            {
                bool is_init_action = false;
                // the last placmeent should always be the current object pose.
                if(p == feasible_intermediate_placement_transforms.size()-1)
                    is_init_action = true;

                for(int g = 0; g < regrasp_ids[p].size(); g++)
                {
                    if(regrasp_types[p][g] == 0) // this grasp pose can be used to lift the object.
                    {
                        num_of_lifting_grasps++;
                        for(int k = 0; k < lifting_motions[p][g].size(); k++)
                        {
                            task_planner.addActionBetweenFoliations(lifting_motions[p][g][k], 0, regrasp_ids[p][g], 1, regrasp_ids[p][g], -0.005);
                            task_planner.addGoalInManifolds(regrasp_ids[p][g], 1, 1.0);
                        }
                    }
                    for(int k = 0; k < approach_motions[p][g].size(); k++)
                    {
                        task_planner.addActionBetweenManifolds(approach_motions[p][g][k], p, regrasp_ids[p][g], 0, is_init_action, -0.005);
                    }
                }
            }

            if(num_of_lifting_grasps == 0) // if there is not grasp pose to lift up the object, then return failure.
            {
                if(num_of_trials == 0){
                    ROS_INFO("no way to lift the object");
                    return 0;
                }
                else{
                    num_of_trials--;
                    retry = true;
                    hasTargetObject = true;
                    ROS_INFO("no way to lift the object, try again");
                    continue;
                }
            }

            // visualize the all re-grasping poses.
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

            if (!grasp_poses_visualizer.call(regrasp_poses_visualize_srv))
            {
                ROS_ERROR("Failed to call service visualize_regrasp");
                return 1;
            }

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

                intermediate_placements_visualize_srv.request.intermediate_placement_depths.push_back(obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x);
                intermediate_placements_visualize_srv.request.intermediate_placement_widths.push_back(obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y);
                intermediate_placements_visualize_srv.request.intermediate_placement_heights.push_back(obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.z);
            }

            if(!intermediate_placements_visualizer.call(intermediate_placements_visualize_srv))
            {
                ROS_ERROR("Failed to call service visualize_intermediate_placements");
                return 1;
            }

            //------------------------------------------------------------------------------------------
            // initialize the constraint for each manifold
            std::vector<moveit_msgs::Constraints> manipulation_manifold_constraints;
            std::vector<moveit_msgs::Constraints> intermediate_manifold_constraints;
            std::vector<moveit_msgs::Constraints> placing_manifold_constraints;
            for(int i = 0; i < grasp_transforms.size(); i++)
            {
                moveit_msgs::Constraints constraints;
                constraints.name = "use_equality_constraints";

                geometry_msgs::Pose in_hand_pose;
                transformTFToGeoPose(grasp_transforms[i].inverse(), in_hand_pose);
                constraints.in_hand_pose = in_hand_pose;

                // define the orientation constraint on the object
                moveit_msgs::OrientationConstraint orientation_constraint;
                orientation_constraint.parameterization = moveit_msgs::OrientationConstraint::ROTATION_VECTOR;
                orientation_constraint.header.frame_id = "base_link";
                orientation_constraint.header.stamp = ros::Time(0);
                orientation_constraint.link_name = "wrist_roll_link";
                geometry_msgs::Quaternion constrained_quaternion;
                constrained_quaternion.x = target_object_transform.getRotation().x();
                constrained_quaternion.y = target_object_transform.getRotation().y();
                constrained_quaternion.z = target_object_transform.getRotation().z();
                constrained_quaternion.w = target_object_transform.getRotation().w();
                orientation_constraint.orientation = constrained_quaternion;
                orientation_constraint.weight = 1.0;
                orientation_constraint.absolute_x_axis_tolerance = 2 * 3.1415;
                orientation_constraint.absolute_y_axis_tolerance = 0.05;
                orientation_constraint.absolute_z_axis_tolerance = 0.05;
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
                // bounding_region.dimensions[bounding_region.BOX_X] = 0.0005; // for x
                // bounding_region.dimensions[bounding_region.BOX_Y] = 2000; // for y 
                // bounding_region.dimensions[bounding_region.BOX_Z] = 2000; // for z
                // // If you want to use plane constraint, you should set 0.0005 for it.

                bounding_region.dimensions[bounding_region.BOX_X] = 0.0005; // for x
                bounding_region.dimensions[bounding_region.BOX_Y] = table_srv.response.depth * 0.9; // for y 
                bounding_region.dimensions[bounding_region.BOX_Z] = table_srv.response.width * 0.9; // for z
                // If you want to use plane constraint, you should set 0.0005 for it.

                // geometry_msgs::Pose target_object_pose;
                // transformTFToGeoPose(target_object_transform, target_object_pose);

                // try to find the proper position constraint
                tf::Transform position_constraint_transform(table_transform.inverse() * target_object_transform);
                position_constraint_transform.setOrigin(tf::Vector3(0, 0, position_constraint_transform.getOrigin().getZ()));
                position_constraint_transform = table_transform * position_constraint_transform;

                tf::Matrix3x3 position_constraint_transform_matrix(table_transform.getBasis().getColumn(2).getX(),table_transform.getBasis().getColumn(1).getX(),-table_transform.getBasis().getColumn(0).getX(),
                                                                table_transform.getBasis().getColumn(2).getY(),table_transform.getBasis().getColumn(1).getY(),-table_transform.getBasis().getColumn(0).getY(),
                                                                table_transform.getBasis().getColumn(2).getZ(),table_transform.getBasis().getColumn(1).getZ(),-table_transform.getBasis().getColumn(0).getZ());
                position_constraint_transform.setBasis(position_constraint_transform_matrix);

                geometry_msgs::Pose position_constraint_pose;
                transformTFToGeoPose(position_constraint_transform, position_constraint_pose);
        
                position_constraint.constraint_region.primitives.push_back(bounding_region);
                // position_constraint.constraint_region.primitive_poses.push_back(target_object_pose);
                position_constraint.constraint_region.primitive_poses.push_back(position_constraint_pose);
                constraints.position_constraints.push_back(position_constraint);
                manipulation_manifold_constraints.push_back(constraints);

                //////////////////////////////////////////////////////////////////////////////////////

                moveit_msgs::Constraints placing_constraints;
                placing_constraints.name = "use_equality_constraints";
                placing_constraints.in_hand_pose = in_hand_pose;

                // define the orientation constraint on the object with larger threshold
                orientation_constraint.absolute_x_axis_tolerance = 2 * 3.1415;
                orientation_constraint.absolute_y_axis_tolerance = 0.1;
                orientation_constraint.absolute_z_axis_tolerance = 0.1;
                placing_constraints.orientation_constraints.push_back(orientation_constraint);
                placing_manifold_constraints.push_back(placing_constraints);
            }

            for(int i = 0; i < feasible_intermediate_placement_transforms.size(); i++)
            {
                // in the intermediate manifold, there should be no constraint.
                moveit_msgs::Constraints constraints;
                constraints.name = "no_constraints";

                // set in hand pose to be identity
                geometry_msgs::Pose in_hand_pose;
                in_hand_pose.position.x = 0;
                in_hand_pose.position.y = 0;
                in_hand_pose.position.z = 0;

                in_hand_pose.orientation.x = 0;
                in_hand_pose.orientation.y = 0;
                in_hand_pose.orientation.z = 0;
                in_hand_pose.orientation.w = 1;
                
                constraints.in_hand_pose = in_hand_pose;

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
                bounding_region.dimensions[bounding_region.BOX_Z] = 2000; // for z

                geometry_msgs::Pose target_object_pose;
                transformTFToGeoPose(target_object_transform, target_object_pose);

                position_constraint.constraint_region.primitives.push_back(bounding_region);
                position_constraint.constraint_region.primitive_poses.push_back(target_object_pose);
                constraints.position_constraints.push_back(position_constraint);

            
                intermediate_manifold_constraints.push_back(constraints);
            }
            std::cout << "init constraints done" << std::endl;

            // init the target object as the collision object for re-grasping without pose.
            collision_object_target.primitives.clear();
            shape_msgs::SolidPrimitive target_primitive;
            target_primitive.type = target_primitive.BOX;
            target_primitive.dimensions.resize(3);
            target_primitive.dimensions[0] = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x;
            target_primitive.dimensions[1] = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y;
            target_primitive.dimensions[2] = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.z;
            collision_object_target.primitives.push_back(target_primitive);

            std::cout << "init all collision objects done" << std::endl;

            // get the current joint positions of the robot
            std::vector<double> current_joint_positions_double;
            std::vector<float> current_joint_positions_float;
            current_state.copyJointGroupPositions(joint_model_group, current_joint_positions_double);
            for(int i = 0; i < current_joint_positions_double.size(); i++)
                current_joint_positions_float.push_back((float)current_joint_positions_double[i]);
            
            // construct the task planner graph
            task_planner.constructMDPGraph();

            // setup the motion planner
            move_group.setPlannerId("CLazyPRMConfigDefault");
            move_group.setPlanningTime(0.5);

            bool cannotfindplan = false;

            for(int planning_verify_number = 0; planning_verify_number < 50; planning_verify_number++)
            {
                // run policy interation for the task.
                task_planner.policyIteration();
                std::cout << "verify the action sequence at time " << planning_verify_number << std::endl;
                
                // input the current joint position and plan for the action sequence 
                ActionSequence action_sequence;
                if(not task_planner.planActions(action_sequence, current_joint_positions_float))
                {
                    cannotfindplan = true;
                    break;
                }

                // init the current state of the robot
                current_state = *(move_group.getCurrentState());

                // reset the total trajectory.
                robot_action_trajectory_execution_list.clear();
                need_to_reanalyze = true;

                // check the action sequence
                for(int i = 0; i < action_sequence.getActionSize(); i++)
                {
                    // get the current motion task
                    MotionTask m = action_sequence.getActionTaskAt(i);
                    std::cout << "-------------------------- motion task " << i;
                    if(m.is_in_manipulation_manifold)
                    {
                        std::cout << " in manipulation manipulation manifold with id " << m.manifold_id << " of foliation " << m.foliation_id << std::endl;
                    }
                    else
                    {
                        std::cout << " in intermediate placement manifold with id " << m.manifold_id << " of foliation " << m.foliation_id << std::endl;
                    }

                    if(m.target_joint_values.size() == 0)
                    { // plan for the placing action
                        collision_object_table.operation = collision_object_table.ADD;
                        planning_scene_interface.applyCollisionObject(collision_object_table);
                        // calculate current target object pose.
                        Eigen::Isometry3d current_in_hand_pose;
                        tf::transformTFToEigen(grasp_transforms[m.manifold_id].inverse(), current_in_hand_pose);

                        // 1. attach the object to the end effector in the planning scene.
                        current_state.attachBody("target_object", current_in_hand_pose, target_object_shapes, shape_poses, std::vector<std::string>{"l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"}, "wrist_roll_link");
                        
                        // 2. set the proper planner in the move group
                        move_group.setActionWithId("place", m.manifold_id);

                        // 3. set the constraint in this manifold
                        move_group.setPathConstraints(placing_manifold_constraints[m.manifold_id]);
                        move_group.setInHandPose(placing_manifold_constraints[m.manifold_id].in_hand_pose);

                        // 4. set the start and target joint values
                        move_group.setStartState(current_state);

                        moveit::planning_interface::MoveGroupInterface::Plan place_plan;
                            
                        move_group.setPositionTarget(0.32, -0.5, 0.9);

                        // 5. plan and execute the motion
                        bool success = (move_group.plan(place_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        
                        // reset the move group
                        move_group.clearPathConstraints();
                        move_group.clearAction();
                        move_group.clearPoseTargets();
                        move_group.clearInHandPose();
                        
                        current_state.clearAttachedBody("target_object");
                        collision_object_table.operation = collision_object_table.REMOVE;
                        planning_scene_interface.applyCollisionObject(collision_object_table);

                        if(!success)
                        {
                            std::cout << "place plan failed" << std::endl;
                            
                            break;
                        }

                        std::cout << "place plan success" << std::endl;
                        robot_trajectory::RobotTrajectory place_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, place_plan.trajectory_);
                        action_sequence.setSolutionForActionTaskAt(i, place_plan.trajectory_, placing_manifold_constraints[m.manifold_id].in_hand_pose);
                        robot_action_trajectory_execution_list.push_back(std::make_pair("arm", place_trajectory));                
                        current_state = place_trajectory.getLastWayPoint();

                        // get the end effector pose of current state
                        tf::Transform current_hand_transform;
                        tf::transformEigenToTF(current_state.getGlobalLinkTransform("wrist_roll_link"), current_hand_transform);
                        geometry_msgs::Pose place_down_grasp_pose;
                        transformTFToGeoPose(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0, 0, -0.1)) * current_hand_transform, place_down_grasp_pose);

                        // need to add place down action
                        move_group.setStartState(current_state);
                        moveit_msgs::RobotTrajectory place_down_trajectory_msg;    
                        if(move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{place_down_grasp_pose}, 0.005, 5.0, place_down_trajectory_msg, true) < 0.95)
                            continue;
                        robot_trajectory::RobotTrajectory place_down_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, place_down_trajectory_msg);
                        robot_action_trajectory_execution_list.push_back(std::make_pair("arm", place_down_trajectory));
                        current_state = place_down_trajectory.getLastWayPoint();

                        // open the gripper
                        end_effector_move_group.setStartState(current_state);
                        for(int i = 0; i < finger_joint_names.size(); i++)
                            end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.04);
                        moveit::planning_interface::MoveGroupInterface::Plan open_gripper_plan;
                        if(end_effector_move_group.plan(open_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                            continue;
                        robot_trajectory::RobotTrajectory open_gripper_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, open_gripper_plan.trajectory_);
                        robot_action_trajectory_execution_list.push_back(std::make_pair("open", open_gripper_trajectory));
                        current_state = open_gripper_trajectory.getLastWayPoint();


                        continue;
                    }

                    // if the current task has solution, then continue.
                    if(action_sequence.hasSolutionAt(i))
                    {
                        // reuse the solution from the task graph
                        robot_trajectory::RobotTrajectory solution_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, m.solution_trajectory);
                        robot_action_trajectory_execution_list.push_back(std::make_pair("arm", solution_trajectory));
                        current_state = solution_trajectory.getLastWayPoint();

                        // if action is to lift the object then do not open gripper
                        if(!action_sequence.isToNextFoliationAt(i))
                        {
                            // need to open gripper no matter sliding or re-grasping------------------------------
                            end_effector_move_group.setStartState(current_state);
                            for(int i = 0; i < finger_joint_names.size(); i++)
                                end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.04);
                            
                            moveit::planning_interface::MoveGroupInterface::Plan open_gripper_plan;
                            if(end_effector_move_group.plan(open_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                                break;
                            robot_trajectory::RobotTrajectory open_gripper_trajectory = 
                                            robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, open_gripper_plan.trajectory_);
                            robot_action_trajectory_execution_list.push_back(std::make_pair("open", open_gripper_trajectory));
                            current_state = open_gripper_trajectory.getLastWayPoint();
                        }

                        if(!m.is_in_manipulation_manifold)// if we want to re-grasping, then we need to close the gripper
                        {
                            // get the cartesian trajectory
                            robot_trajectory::RobotTrajectory cartesian_trajectory = 
                                            robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, action_sequence.getCartesianMotionAt(i));
                            robot_action_trajectory_execution_list.push_back(std::make_pair("arm", cartesian_trajectory));
                            current_state = cartesian_trajectory.getLastWayPoint();

                            // need to close gripper
                            end_effector_move_group.setStartState(current_state);
                            for(int i = 0; i < finger_joint_names.size(); i++)
                                end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.00);

                            moveit::planning_interface::MoveGroupInterface::Plan close_gripper_plan;
                            if(end_effector_move_group.plan(close_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                                break;

                            robot_trajectory::RobotTrajectory close_gripper_trajectory = 
                                            robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, close_gripper_plan.trajectory_);
                            robot_action_trajectory_execution_list.push_back(std::make_pair("close", close_gripper_trajectory));
                            current_state = close_gripper_trajectory.getLastWayPoint();
                        }
                        else{
                            // get the cartesian trajectory
                            robot_trajectory::RobotTrajectory cartesian_trajectory = 
                                            robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, action_sequence.getCartesianMotionAt(i));

                            if(!action_sequence.isToNextFoliationAt(i)){
                                robot_action_trajectory_execution_list.push_back(std::make_pair("after_slide", cartesian_trajectory));
                                if(need_to_reanalyze) // only record the object transform at the before the first regrasping
                                {
                                    tf::Transform current_in_hand_transform;
                                    GeoPoseTotransformTF(action_sequence.getInHandPoseAt(i), current_in_hand_transform);
                                    tf::Transform current_hand_transform;
                                    tf::transformEigenToTF(current_state.getGlobalLinkTransform("wrist_roll_link"), current_hand_transform);
                                    target_object_transform_during_regrasp = current_hand_transform * current_in_hand_transform;
                                    need_to_reanalyze = false;
                                }
                            }
                            else
                                robot_action_trajectory_execution_list.push_back(std::make_pair("lift", cartesian_trajectory));
                            
                            current_state = cartesian_trajectory.getLastWayPoint();
                        }
                        continue;
                    }
                
                    if(m.is_in_manipulation_manifold) // if current task is in manipulation manifold
                    {
                        // calculate current target object pose.
                        Eigen::Isometry3d current_in_hand_pose;
                        tf::transformTFToEigen(grasp_transforms[m.manifold_id].inverse(), current_in_hand_pose);
                        
                        // 2. set the proper planner in the move group
                        move_group.setActionWithId("slide", m.manifold_id);

                        // 3. set the constraint in this manifold
                        move_group.setPathConstraints(manipulation_manifold_constraints[m.manifold_id]);
                        move_group.setInHandPose(manipulation_manifold_constraints[m.manifold_id].in_hand_pose);

                        // compare the goal joint values with the current joint values
                        bool is_goal_joint_values_same = true;
                        std::vector<double> current_state_joint_positions_double;
                        current_state.copyJointGroupPositions(joint_model_group, current_state_joint_positions_double);
                        for(int j = 0; j < current_state_joint_positions_double.size(); j++)
                        {
                            if(std::fabs(current_state_joint_positions_double[j] - m.target_joint_values[j]) > 0.0000001)
                            {
                                is_goal_joint_values_same = false;
                                break;
                            }
                        }

                        moveit::planning_interface::MoveGroupInterface::Plan slide_plan;
                        if(!is_goal_joint_values_same)
                        {
                            std::vector<double> target_joint_positions_double;
                            for(int j = 0; j < m.target_joint_values.size(); j++){
                                target_joint_positions_double.push_back((double)m.target_joint_values[j]);
                            }
                                
                            move_group.setJointValueTarget(target_joint_positions_double);

                            current_state.attachBody("target_object", current_in_hand_pose, target_object_shapes, shape_poses, std::vector<std::string>{"l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"}, "wrist_roll_link");
                            move_group.setStartState(current_state);

                            // 5. plan and execute the motion
                            bool success = (move_group.plan(slide_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                            current_state.clearAttachedBody("target_object");
                            
                            // reset the move group
                            move_group.clearPathConstraints();
                            move_group.clearAction();
                            // move_group.clearExperience();
                            move_group.clearPoseTargets();
                            move_group.clearInHandPose();

                            if(!success)
                            {
                                std::cout << "slide plan failed" << std::endl;
                                break;
                            }
                        }
                        else{
                            // if the goal joint values are the same as the current joint values, then we don't need to plan the motion.
                            // set the trajectory to current state
                            robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).addSuffixWayPoint(current_state, 0.01).getRobotTrajectoryMsg(slide_plan.trajectory_);
                        }

                        std::cout << "slide plan success" << std::endl;
                        robot_trajectory::RobotTrajectory slide_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, slide_plan.trajectory_);
                        action_sequence.setSolutionForActionTaskAt(i, slide_plan.trajectory_, manipulation_manifold_constraints[m.manifold_id].in_hand_pose);
                        robot_action_trajectory_execution_list.push_back(std::make_pair("arm", slide_trajectory));                
                        current_state = slide_trajectory.getLastWayPoint();

                        // need to open gripper
                        if(!action_sequence.isToNextFoliationAt(i))
                        {
                            end_effector_move_group.setStartState(current_state);
                            for(int i = 0; i < finger_joint_names.size(); i++)
                                end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.04);
                            moveit::planning_interface::MoveGroupInterface::Plan open_gripper_plan;
                            if(end_effector_move_group.plan(open_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                            {
                                std::cout << "open gripper plan failed" << std::endl;
                                break;
                            }
                            robot_trajectory::RobotTrajectory open_gripper_trajectory = 
                                            robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, open_gripper_plan.trajectory_);
                            robot_action_trajectory_execution_list.push_back(std::make_pair("open", open_gripper_trajectory));
                            current_state = open_gripper_trajectory.getLastWayPoint();

                            // 6. execute the cartesian motion
                            robot_trajectory::RobotTrajectory cartesian_trajectory = 
                                            robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, action_sequence.getCartesianMotionAt(i));

                            // 7. add the cartesian trajectory into the total trajectory.
                            robot_action_trajectory_execution_list.push_back(std::make_pair("after_slide", cartesian_trajectory));
                            if(need_to_reanalyze) // only record the object transform at the before the first regrasping
                            {
                                tf::Transform current_in_hand_transform;
                                GeoPoseTotransformTF(manipulation_manifold_constraints[m.manifold_id].in_hand_pose, current_in_hand_transform);
                                tf::Transform current_hand_transform;
                                tf::transformEigenToTF(current_state.getGlobalLinkTransform("wrist_roll_link"), current_hand_transform);
                                target_object_transform_during_regrasp = current_hand_transform * current_in_hand_transform;
                                need_to_reanalyze = false;
                            }
                            current_state = cartesian_trajectory.getLastWayPoint();
                        }
                        else{
                            // if the action is to the next foliation, then we need to lift the object.
                            // 6. execute the cartesian motion
                            robot_trajectory::RobotTrajectory cartesian_trajectory = 
                                            robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, action_sequence.getCartesianMotionAt(i));

                            // 7. add the cartesian trajectory into the total trajectory.
                            robot_action_trajectory_execution_list.push_back(std::make_pair("lift", cartesian_trajectory));
                            current_state = cartesian_trajectory.getLastWayPoint();
                        }
                    }
                    else
                    {
                        // execute the re-grasping task
                        // 1. add the object and table into the planning scene
                        geometry_msgs::Pose target_object_pose;
                        target_object_pose.position.x = feasible_intermediate_placement_transforms[m.manifold_id].getOrigin().x();
                        target_object_pose.position.y = feasible_intermediate_placement_transforms[m.manifold_id].getOrigin().y();
                        target_object_pose.position.z = feasible_intermediate_placement_transforms[m.manifold_id].getOrigin().z();

                        target_object_pose.orientation.x = feasible_intermediate_placement_transforms[m.manifold_id].getRotation().x();
                        target_object_pose.orientation.y = feasible_intermediate_placement_transforms[m.manifold_id].getRotation().y();
                        target_object_pose.orientation.z = feasible_intermediate_placement_transforms[m.manifold_id].getRotation().z();
                        target_object_pose.orientation.w = feasible_intermediate_placement_transforms[m.manifold_id].getRotation().w();
                        
                        collision_object_target.pose = target_object_pose;
                        collision_object_target.operation = collision_object_target.ADD;
                        planning_scene_interface.applyCollisionObject(collision_object_target);

                        collision_object_table.operation = collision_object_table.ADD;
                        planning_scene_interface.applyCollisionObject(collision_object_table);

                        // 2. set the proper planner in the move group
                        move_group.setActionWithId("regrasp", m.manifold_id);

                        // 3. set the constraint in this manifold
                        move_group.setPathConstraints(intermediate_manifold_constraints[m.manifold_id]);
                        move_group.setInHandPose(intermediate_manifold_constraints[m.manifold_id].in_hand_pose);

                        // 4. set the start and target joint values
                        move_group.setStartState(current_state);

                        std::vector<double> target_joint_positions_double;
                        for(int j = 0; j < m.target_joint_values.size(); j++)
                            target_joint_positions_double.push_back((double)m.target_joint_values[j]);
                        move_group.setJointValueTarget(target_joint_positions_double);

                        // 5. plan and execute the motion
                        move_group.setMaxVelocityScalingFactor(0.6);
                        moveit::planning_interface::MoveGroupInterface::Plan regrasp_plan;
                        bool success = (move_group.plan(regrasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        move_group.setMaxVelocityScalingFactor(0.1);

                        move_group.clearPathConstraints();
                        move_group.clearInHandPose();
                        move_group.clearAction();
                        // move_group.clearExperience();
                        move_group.clearPoseTargets();
                        move_group.clearInHandPose();

                        // 8. remove the object and the table from the planning scene
                        collision_object_target.operation = collision_object_target.REMOVE;
                        planning_scene_interface.applyCollisionObject(collision_object_target);
                        collision_object_table.operation = collision_object_table.REMOVE;
                        planning_scene_interface.applyCollisionObject(collision_object_table);

                        if(!success)
                        {
                            std::cout << "regrasp plan failed" << std::endl;
                            break;
                        }
                        std::cout << "regrasp plan success" << std::endl;
                        robot_trajectory::RobotTrajectory regrasp_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, regrasp_plan.trajectory_);

                        // set the solution in the action sequence
                        action_sequence.setSolutionForActionTaskAt(i, regrasp_plan.trajectory_, intermediate_manifold_constraints[m.manifold_id].in_hand_pose);
                        robot_action_trajectory_execution_list.push_back(std::make_pair("arm", regrasp_trajectory));
                        current_state = regrasp_trajectory.getLastWayPoint();

                        // need to open gripper
                        end_effector_move_group.setStartState(current_state);
                        for(int i = 0; i < finger_joint_names.size(); i++)
                            end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.04);

                        moveit::planning_interface::MoveGroupInterface::Plan open_gripper_plan;
                        if(end_effector_move_group.plan(open_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                            break;
                        robot_trajectory::RobotTrajectory open_gripper_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, open_gripper_plan.trajectory_);
                        robot_action_trajectory_execution_list.push_back(std::make_pair("open", open_gripper_trajectory));
                        current_state = open_gripper_trajectory.getLastWayPoint();

                        // 6. execute the cartesian motion
                        robot_trajectory::RobotTrajectory cartesian_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group).setRobotTrajectoryMsg(current_state, action_sequence.getCartesianMotionAt(i));

                        // 7. add the re-grasp trajectory into the total trajectory.
                        robot_action_trajectory_execution_list.push_back(std::make_pair("arm", cartesian_trajectory));
                        current_state = cartesian_trajectory.getLastWayPoint();

                        // need to close gripper
                        end_effector_move_group.setStartState(current_state);
                        for(int i = 0; i < finger_joint_names.size(); i++)
                            end_effector_move_group.setJointValueTarget(finger_joint_names[i], 0.00);
                        moveit::planning_interface::MoveGroupInterface::Plan close_gripper_plan;
                        if(end_effector_move_group.plan(close_gripper_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
                            break;
                        robot_trajectory::RobotTrajectory close_gripper_trajectory = 
                                        robot_trajectory::RobotTrajectory(kinematic_model, end_effector_joint_model_group).setRobotTrajectoryMsg(current_state, close_gripper_plan.trajectory_);
                        robot_action_trajectory_execution_list.push_back(std::make_pair("close", close_gripper_trajectory));
                        current_state = close_gripper_trajectory.getLastWayPoint();                
                    }
                }
                // check current action sequence
                bool solution_is_good = true;
                std::cout << "** check the solution on action sequence **" << std::endl;
                for(int i = 0; i < action_sequence.getActionSize(); i++)
                {
                    if(action_sequence.getActionTaskAt(i).solution_trajectory.joint_trajectory.points.size() == 0)
                    { // there is no solution in this action of the solution sequence
                        solution_is_good = false;
                        break;
                    }
                }

                // if the solution is infeasible, then we need to re-plan the action sequence by updating the task graph.
                if(not solution_is_good)
                {
                    std::cout << "solution is not good, ";
                    task_planner.updateTaskPlanner(action_sequence);
                    robot_action_trajectory_execution_list.clear();
                    std::cout << "ready to re-plan the action sequence" << std::endl;
                }
                else
                {
                    std::cout << "solution is good, break the loop" << std::endl;

                    // clean the planner structure.
                    move_group.setPlannerId("RRTConnectkConfigDefault");
                    move_group.setStartState(*(move_group.getCurrentState()));

                    move_group.setJointValueTarget(home_joint_values);
                    move_group.setCleanPlanningContextFlag(true);
                    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
                    move_group.plan(home_plan);

                    break;
                }
            }
            if(cannotfindplan)
            {
                if(num_of_trials == 0){
                    ROS_INFO("no plan found");
                    return 0;
                }
                else{
                    num_of_trials--;
                    retry = true;
                    hasTargetObject = true;
                    ROS_INFO("no plan found, try again");
                    continue;
                }
            }
        }

        // where there is no solution, we need to reset the arm to home position.
        if(robot_action_trajectory_execution_list.size() == 0)
        {
            // reset the arm to home position
            move_group.setPlannerId("RRTConnectkConfigDefault");
            move_group.setStartState(*(move_group.getCurrentState()));

            move_group.setJointValueTarget(home_joint_values);
            move_group.setCleanPlanningContextFlag(true);
            moveit::planning_interface::MoveGroupInterface::Plan home_plan;
            move_group.plan(home_plan);

            planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

            std::cout << "no solution is found" << std::endl;
            return 0;
        }

        if(is_execute)
        {
            // wait for user input
            std::cout << "press enter to execute the trajectory" << std::endl;
            std::cin.get();


            // for simulation
            // actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_trajectory_action_("/gripper_controller/follow_joint_trajectory", true);
            // we may need to wait for a while for the action server to be ready
            // while(!gripper_trajectory_action_.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the gripper_trajectory_action_ action server to come up");
            // }
            for(auto iter: robot_action_trajectory_execution_list)
            {
                if(iter.first.compare("arm") == 0){
                    std::cout << "move arm" << std::endl;

                    // remove duplicate waypoints
                    for(int t = 0; t < iter.second.getWayPointCount(); t++)
                        if(iter.second.getWayPointDurationFromPrevious(t) == 0.0)
                            iter.second.setWayPointDurationFromPrevious(t, 0.05);
                        
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    iter.second.getRobotTrajectoryMsg(plan.trajectory_);
                    move_group.execute(plan);
                }
                else if(iter.first.compare("lift") == 0){

                    std::cout << "move arm to lift object" << std::endl;
                    // wait for 1 s
                    ros::Duration(1.0).sleep();
                    // remove duplicate waypoints
                    for(int t = 0; t < iter.second.getWayPointCount(); t++)
                        if(iter.second.getWayPointDurationFromPrevious(t) == 0.0)
                            iter.second.setWayPointDurationFromPrevious(t, 0.05);
                        
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    iter.second.getRobotTrajectoryMsg(plan.trajectory_);
                    move_group.execute(plan);

                }
                else if(iter.first.compare("after_slide") == 0){

                    std::cout << "move arm" << std::endl;
                    // remove duplicate waypoints
                    for(int t = 0; t < iter.second.getWayPointCount(); t++)
                        if(iter.second.getWayPointDurationFromPrevious(t) == 0.0)
                            iter.second.setWayPointDurationFromPrevious(t, 0.05);
                        
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    iter.second.getRobotTrajectoryMsg(plan.trajectory_);
                    move_group.execute(plan);

                    if(re_analyze){ // need to reanalyze the target object before re-grasping.
                        geometry_msgs::Pose target_object_pose;
                        transformTFToGeoPose(target_object_transform_during_regrasp, target_object_pose);
                        collision_object_target.pose = target_object_pose;
                        collision_object_target.operation = collision_object_target.ADD;
                        planning_scene_interface.applyCollisionObject(collision_object_target);

                        collision_object_table.operation = collision_object_table.ADD;
                        planning_scene_interface.applyCollisionObject(collision_object_table);
                        std::cout << "move arm before re-grasping" << std::endl;
                        move_group.setPlannerId("RRTConnectkConfigDefault");
                        move_group.setStartState(*(move_group.getCurrentState()));

                        // move_group.setPositionTarget(0.248, -0.76, 0.721);
                        move_group.setJointValueTarget(home_joint_values);
                        move_group.setCleanPlanningContextFlag(true);
                        move_group.setMaxVelocityScalingFactor(0.6);
                        moveit::planning_interface::MoveGroupInterface::Plan re_analyze_plan;
                        // std::vector<moveit::planning_interface::MoveGroupInterface::MotionEdge> experience;
                        // bool success = (move_group.plan(re_analyze_plan, experience) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        bool success = (move_group.plan(re_analyze_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        move_group.setMaxVelocityScalingFactor(0.1);

                        move_group.clearPathConstraints();
                        move_group.clearInHandPose();
                        move_group.clearAction();
                        // move_group.clearExperience();
                        move_group.clearPoseTargets();
                        move_group.clearInHandPose();

                        if(!success)
                        {
                            std::cout << "fail at re-analyze before re-grasping" << std::endl;
                            return 0;
                        }

                        move_group.execute(re_analyze_plan);

                        planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

                        target_object_transform = target_object_transform_during_regrasp;
                        hasTargetObject = true;

                        break;
                    }
                }
                else if(iter.first.compare("open") == 0){
                    std::cout << "open gripper" << std::endl;
                    // control_msgs::FollowJointTrajectoryGoal goal;
                    // goal.trajectory.header.stamp = ros::Time::now();
                    // goal.trajectory.joint_names.push_back("r_gripper_finger_joint");
                    // trajectory_msgs::JointTrajectoryPoint point;
                    // point.positions.push_back(0.04);
                    // point.time_from_start = ros::Duration(1.0);
                    // goal.trajectory.points.push_back(point);
                    // gripper_trajectory_action_.sendGoalAndWait(goal, ros::Duration(3.0));

                    // open gripper in the real world
                    std::vector<double> open_finger_joint_values = {0.05, 0.05};
                    end_effector_move_group.setJointValueTarget(finger_joint_names, open_finger_joint_values);
                    end_effector_move_group.move();
                    ros::Duration(1.0).sleep();
                }
                else if(iter.first.compare("close") == 0){
                    std::cout << "close gripper" << std::endl;
                    // control_msgs::FollowJointTrajectoryGoal goal;
                    // goal.trajectory.header.stamp = ros::Time::now();
                    // goal.trajectory.joint_names.push_back("r_gripper_finger_joint");
                    // trajectory_msgs::JointTrajectoryPoint point;
                    // point.positions.push_back(-0.04);
                    // point.time_from_start = ros::Duration(1.0);
                    // goal.trajectory.points.push_back(point);
                    // gripper_trajectory_action_.sendGoalAndWait(goal, ros::Duration(3.0));
                    // // get current finger joint value
                    // std::vector<double> finger_joint_values;
                    // move_group.getCurrentState()->copyJointGroupPositions(end_effector_joint_model_group, finger_joint_values);
                    // // resend the goal
                    // goal.trajectory.points[0].positions[0] = finger_joint_values[0];
                    // goal.trajectory.points[0].time_from_start = ros::Duration(0.5);
                    // gripper_trajectory_action_.sendGoalAndWait(goal, ros::Duration(1.5));

                    // close gripper in the real world
                    // std::vector<double> close_finger_joint_values = {0.028, 0.028}; // for coco can
                    std::vector<double> close_finger_joint_values = {0.012, 0.012}; // for coffee handle
                    end_effector_move_group.setJointValueTarget(finger_joint_names, close_finger_joint_values);
                    end_effector_move_group.move();
                    ros::Duration(1.0).sleep();
                }
                else{
                    std::cout << "unknown action" << std::endl;
                    return 0;
                }
            }

        }
        else{
            // init the moveit visual tools for visualization
            moveit_visual_tools::MoveItVisualToolsPtr trajectory_visuals = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link");
            robot_trajectory::RobotTrajectory total_trajectory_for_visual = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);

            // visualize the robot action and trajectory
            for(auto iter: robot_action_trajectory_execution_list)
            {
                if(iter.first.compare("arm") == 0){
                    std::cout << "visual move arm" << std::endl;
                    total_trajectory_for_visual.append(iter.second, 0.01);
                }
                else if(iter.first.compare("after_slide") == 0){
                    std::cout << "visual move arm" << std::endl;
                    total_trajectory_for_visual.append(iter.second, 0.01);
                }
                else if(iter.first.compare("lift") == 0){
                    std::cout << "visual move arm to lift object" << std::endl;
                    total_trajectory_for_visual.append(iter.second, 0.01);
                }
                else if(iter.first.compare("open") == 0){
                    std::cout << "visual open gripper" << std::endl;
                    total_trajectory_for_visual.append(iter.second, 0.01);
                }
                else if(iter.first.compare("close") == 0){
                    std::cout << "visual close gripper" << std::endl;
                    total_trajectory_for_visual.append(iter.second, 0.01);
                }
                else{
                    std::cout << "unknown action" << std::endl;
                    return 0;
                }
            }

            // visualize the trajectory
            total_trajectory_for_visual.setGroupName("arm_with_gripper");
            trajectory_visuals->publishTrajectoryPath(total_trajectory_for_visual);
        }
        // remove all collision objects
        planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
    }while(re_analyze || retry);
    return 0;
}