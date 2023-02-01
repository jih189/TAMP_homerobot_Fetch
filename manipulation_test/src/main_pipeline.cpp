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

#include <rail_segmentation/SearchTable.h>
#include <rail_manipulation_msgs/SegmentObjects.h>

#include <ros_tensorflow_msgs/Predict.h>

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

int main(int argc, char** argv)
{
    static const std::string OBJECT_NAME = "hammer";

    ros::init(argc, argv, "main_pipeline");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // // initialize the moveit interfaces
    // static const std::string PLANNING_GROUP = "arm";
    // static const std::string END_EFFECTOR_PLANNING_GROUP = "gripper";
    // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // moveit::planning_interface::MoveGroupInterface end_effector_move_group(END_EFFECTOR_PLANNING_GROUP);

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    // // get the robot kinematic model
    // robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    // init the state validity checker for the robot
    // ros::ServiceClient validity_client =  node_handle.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");
    // validity_client.waitForExistence();


    //******************************************** search for the table and add it ad the obstacle in the planning scene
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

    //******************************************** search the objects on the table
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
        ROS_INFO("No object found on the table");
        return 0;
    }

    // visualize the obstacle
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

    // need to analyze which obstacle should be moved away before grasping the targect object.
    int last_front_obstacle_id = grasped_object_id;
    int front_obstacle_id = last_front_obstacle_id;
    
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

    std::cout << "the object should be grasped now is: " << front_obstacle_id << std::endl;

    // we need to extract the grasped object point cloud with full point cloud
    // init client for grasp prediction.
    ros::ServiceClient grasp_prediction_client = node_handle.serviceClient<ros_tensorflow_msgs::Predict>("grasp_predict");
    grasp_prediction_client.waitForExistence();

    // need to get the camera pose as well.
    tf::TransformListener listener;
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

    ros_tensorflow_msgs::Predict grasp_prediction_srv;

    grasp_prediction_srv.request.full_point_cloud = table_srv.response.full_point_cloud;
    grasp_prediction_srv.request.segmented_point_cloud = obstacle_srv.response.segmented_objects.objects[front_obstacle_id].point_cloud;
    grasp_prediction_srv.request.camera_stamped_transform = camera_stamped_transform;

    if (not grasp_prediction_client.call(grasp_prediction_srv))
    {
        ROS_ERROR("Failed to call service grasp prediction");
        return 1;
    }

    // find the com of the object.
    /************************************************************************************/
    // get the target object transform
    tf::Transform target_object_transform(tf::Quaternion(obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.orientation.x, 
                                                        obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.orientation.y, 
                                                        obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.orientation.z, 
                                                        obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.orientation.w), 
                                          tf::Vector3(obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.position.x, 
                                                      obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.position.y, 
                                                      obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.pose.pose.position.z));

    tf::Vector3 target_com;
    // get object pose from tf
    std::vector<std::string> object_names = {"can", "book", "bottle", "hammer"};
    for(std::string ob: object_names)
    {   
        try{
            tf::StampedTransform object_transform_temp;
            ros::Time now = ros::Time::now();
            listener.waitForTransform("/base_link", "/" + ob, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/base_link", "/" + ob, ros::Time(0), object_transform_temp);

            tf::Vector3 distanceToCom = target_object_transform.inverse() * object_transform_temp.getOrigin();

            if(distanceToCom.x() < obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.x &&
               distanceToCom.y() < obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.y &&
               distanceToCom.z() < obstacle_srv.response.segmented_objects.objects[front_obstacle_id].bounding_volume.dimensions.z)
            {
                std::cout << "the target object is: " << ob << std::endl;
                target_com.setX(object_transform_temp.getOrigin().x());
                target_com.setY(object_transform_temp.getOrigin().y());
                target_com.setZ(object_transform_temp.getOrigin().z());
                break;
            }
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    /************************************************************************************/

    std::vector<tf::Transform> grasp_transforms_before_clustering;
    std::vector<float> grasp_jawwidths_before_clustering;
    std::vector<int> grasp_types_before_clustering;

    // add the lifting grasp poses first
    for(int i = 0; i < grasp_prediction_srv.response.predicted_grasp_poses.size(); i++){
        if(grasp_prediction_srv.response.scores[i] < 0.5)
            continue;
        tf::Stamped<tf::Transform> predicted_grasp_transform;
        tf::poseStampedMsgToTF(grasp_prediction_srv.response.predicted_grasp_poses[i], predicted_grasp_transform);
        if(lift_torque_test(target_com, 1.0, predicted_grasp_transform))
        {
            grasp_transforms_before_clustering.push_back(target_object_transform.inverse() * predicted_grasp_transform);
            grasp_jawwidths_before_clustering.push_back(0.08);
            grasp_types_before_clustering.push_back(0);
        }
    }

    // then add sliding grasp poses
    for(int i = 0; i < grasp_prediction_srv.response.predicted_grasp_poses.size(); i++){
        if(grasp_prediction_srv.response.scores[i] < 0.5)
            continue;
        tf::Stamped<tf::Transform> predicted_grasp_transform;
        tf::poseStampedMsgToTF(grasp_prediction_srv.response.predicted_grasp_poses[i], predicted_grasp_transform);
        if(!lift_torque_test(target_com, 1.0, predicted_grasp_transform))
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

    // show the grasp prediction result
    ros::ServiceClient regrasp_poses_visualizer = node_handle.serviceClient<manipulation_test::VisualizeRegrasp>("visualize_regrasp");
    regrasp_poses_visualizer.waitForExistence();

    manipulation_test::VisualizeRegrasp regrasp_poses_visualize_srv;
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

        regrasp_poses_visualize_srv.request.grasp_poses.push_back(grasp_pose_msg);
        regrasp_poses_visualize_srv.request.grasp_jawwidths.push_back(grasp_jawwidths[i]);
        regrasp_poses_visualize_srv.request.grasp_types.push_back(grasp_types[i]);
    }

    if (!regrasp_poses_visualizer.call(regrasp_poses_visualize_srv))
    {
        ROS_ERROR("Failed to call service visualize_regrasp");
        return 1;
    }

    return 0;
}
    /*

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
    for(int i = 0; i < 8; i++)
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
            transformTFToKDL(torso_transform.inverse() * intermedaite_placement_transform * actual_grasp_transforms[i], end_effector_pose);
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
            tf::Transform pre_grasp_pose_in_world_frame = intermedaite_placement_transform * actual_grasp_transforms[g] * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.1, 0, 0));
            geometry_msgs::Pose pre_grasp_pose;
            transformTFToGeoPose(pre_grasp_pose_in_world_frame, pre_grasp_pose);

            std::vector<moveit_msgs::RobotTrajectory> current_lifting_motions; // the lifting motions of current grasp pose
            // desired end-effector pose
            KDL::Frame end_effector_pose;
            transformTFToKDL(torso_transform.inverse() * intermedaite_placement_transform * actual_grasp_transforms[g], end_effector_pose);

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

                    // check whether the start state is valid.
                    moveit_msgs::GetStateValidity::Request validity_request;
                    moveit_msgs::GetStateValidity::Response validity_response;
                    robot_state::robotStateToRobotStateMsg(currentState, validity_request.robot_state);
                    validity_request.group_name = PLANNING_GROUP;

                    validity_client.call(validity_request, validity_response);
                    if (validity_response.valid)
                    {
                        // check the cartesian path
                        move_group.setStartState(currentState);
                        moveit_msgs::RobotTrajectory approach_trajectory_msg;

                        double fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{pre_grasp_pose}, 0.005, 5.0, approach_trajectory_msg, false);
    
                        if(fraction >= 0.95)
                        {
                            // check the validity of the end point of the path
                            robot_state::RobotState end_state = currentState;
                            end_state.setVariablePositions(joint_names, approach_trajectory_msg.joint_trajectory.points.back().positions);
                            robot_state::robotStateToRobotStateMsg(end_state, validity_request.robot_state);

                            validity_client.call(validity_request, validity_response);
                            if (validity_response.valid)
                                current_lifting_motions.push_back(approach_trajectory_msg);
                        }
                    }
                        
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
        tf::Transform pre_grasp_pose_in_world_frame = target_object_transform * actual_grasp_transforms[g] * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.1, 0, 0));
        geometry_msgs::Pose pre_grasp_pose;
        transformTFToGeoPose(pre_grasp_pose_in_world_frame, pre_grasp_pose);

        std::vector<moveit_msgs::RobotTrajectory> current_lifting_motions; // the lifting motions of current grasp pose
        // desired end-effector pose
        KDL::Frame end_effector_pose;
        transformTFToKDL(torso_transform.inverse() * target_object_transform * actual_grasp_transforms[g], end_effector_pose);

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

                // check whether the start state is valid.
                moveit_msgs::GetStateValidity::Request validity_request;
                moveit_msgs::GetStateValidity::Response validity_response;
                robot_state::robotStateToRobotStateMsg(currentState, validity_request.robot_state);
                validity_request.group_name = PLANNING_GROUP;

                validity_client.call(validity_request, validity_response);
                if (validity_response.valid)
                {
                    // check the cartesian path
                    move_group.setStartState(currentState);
                    moveit_msgs::RobotTrajectory approach_trajectory_msg;

                    double fraction = move_group.computeCartesianPath(std::vector<geometry_msgs::Pose>{pre_grasp_pose}, 0.005, 5.0, approach_trajectory_msg, false);

                    if(fraction >= 0.95)
                    {
                        // check the validity of the end point of the path
                        robot_state::RobotState end_state = currentState;
                        end_state.setVariablePositions(joint_names, approach_trajectory_msg.joint_trajectory.points.back().positions);
                        robot_state::robotStateToRobotStateMsg(end_state, validity_request.robot_state);

                        validity_client.call(validity_request, validity_response);
                        if (validity_response.valid)
                            current_lifting_motions.push_back(approach_trajectory_msg);
                    }
                }
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
    TaskPlanner task_planner(std::vector<long unsigned int>{feasible_intermediate_placement_transforms.size(), 0}, std::vector<long unsigned int>{actual_grasp_transforms.size(), actual_grasp_transforms.size()});
    // in the second foliation, there is no intermediate placement, so we set it to be zero.

    // add all action motions into the task planner.
    for(int f = 0; f < 1; f++)
    {
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
                        task_planner.addActionBetweenFoliations(lifting_motions[p][g][k], f, regrasp_ids[p][g], f+1, regrasp_ids[p][g], 1.0);
                    }
                }
                for(int k = 0; k < lifting_motions[p][g].size(); k++)
                {
                    task_planner.addActionBetweenManifolds(lifting_motions[p][g][k], p, regrasp_ids[p][g], f, is_init_action, -0.005);
                }
            }
        }
    }

    if(num_of_lifting_grasps == 0) // if there is not grasp pose to lift up the object, then return failure.
    {
        std::cout << "no way to lift the object" << std::endl;
        return 0;
    }

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

    //------------------------------------------------------------------------------------------
    // initialize the constraint for each manifold
    std::cout << "number of manipulation manifolds: " << actual_grasp_transforms.size() << std::endl;
    std::cout << "number of intermediate manifolds: " << feasible_intermediate_placement_transforms.size() << std::endl;
    std::vector<moveit_msgs::Constraints> manipulation_manifold_constraints;
    std::vector<moveit_msgs::Constraints> intermediate_manifold_constraints;
    for(int i = 0; i < actual_grasp_transforms.size(); i++)
    {
        moveit_msgs::Constraints constraints;
        constraints.name = "use_equality_constraints";

        geometry_msgs::Pose in_hand_pose;
        transformTFToGeoPose(actual_grasp_transforms[i].inverse(), in_hand_pose);
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
        orientation_constraint.absolute_y_axis_tolerance = 0.1;
        orientation_constraint.absolute_z_axis_tolerance = 0.1;
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
        bounding_region.dimensions[bounding_region.BOX_X] = 0.0005; // for x
        bounding_region.dimensions[bounding_region.BOX_Y] = 2000; // for y 
        bounding_region.dimensions[bounding_region.BOX_Z] = 2000; // for z
        // If you want to use plane constraint, you should set 0.0005 for it.

        geometry_msgs::Pose target_object_pose;
        transformTFToGeoPose(target_object_transform, target_object_pose);

        position_constraint.constraint_region.primitives.push_back(bounding_region);
        position_constraint.constraint_region.primitive_poses.push_back(target_object_pose);
        constraints.position_constraints.push_back(position_constraint);
        manipulation_manifold_constraints.push_back(constraints);
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

    geometry_msgs::Pose table_pose;
    table_pose.position.x = table_srv.response.center.x;
    table_pose.position.y = table_srv.response.center.y;
    table_pose.position.z = table_srv.response.center.z;

    table_pose.orientation.x = table_srv.response.orientation.x;
    table_pose.orientation.y = table_srv.response.orientation.y;
    table_pose.orientation.z = table_srv.response.orientation.z;
    table_pose.orientation.w = table_srv.response.orientation.w;


    collision_object_table.primitives.push_back(table_primitive);
    collision_object_table.pose = table_pose;

    // init the target object as the collision object for re-grasping without pose.
    moveit_msgs::CollisionObject collision_object_target;
    collision_object_target.header.frame_id = move_group.getPlanningFrame();

    collision_object_target.id =  "target_object";

    shape_msgs::SolidPrimitive target_primitive;
    target_primitive.type = target_primitive.BOX;
    target_primitive.dimensions.resize(3);
    target_primitive.dimensions[0] = obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.x;
    target_primitive.dimensions[1] = obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.y;
    target_primitive.dimensions[2] = obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.z;
    collision_object_target.primitives.push_back(target_primitive);

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
    move_group.setPlanningTime(5.0);

    // initialize the target object as the attached object.
    shapes::Shape* target_object_shape = new shapes::Box(obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.x,
                                                        obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.y,
                                                        obstacle_srv.response.segmented_objects.objects[grasped_object_id].bounding_volume.dimensions.z);
    std::vector<shapes::ShapeConstPtr> target_object_shapes;
    target_object_shapes.push_back(shapes::ShapeConstPtr(target_object_shape));
    EigenSTL::vector_Isometry3d shape_poses{Eigen::Isometry3d::Identity()};

    // init the moveit visual tools for visualization
    moveit_visual_tools::MoveItVisualToolsPtr trajectory_visuals = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link");
    moveit_visual_tools::MoveItVisualToolsPtr robot_visuals = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link", "/moveit_visual_tools");
    robot_visuals->loadPlanningSceneMonitor();
    robot_visuals->loadMarkerPub(true);
    robot_visuals->loadRobotStatePub("joint_states");
    robot_visuals->setManualSceneUpdating();

    // init the final robot trajectory for visualization later.
    robot_trajectory::RobotTrajectory total_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);    

    for(int planning_verify_number = 0; planning_verify_number < 30; planning_verify_number++)
    {
        // run policy interation for the task.
        task_planner.policyIteration();
        
        // input the current joint position and plan for the action sequence 
        ActionSequence action_sequence;
        if(not task_planner.planActions(action_sequence, current_joint_positions_float))
        {
            std::cout << "no plan found" << std::endl;
            return 0;
        }

        std::cout << "verify the action sequence at time " << planning_verify_number << std::endl;

        // init the current state of the robot
        current_state = *(move_group.getCurrentState());

        // reset the total trajectory.
        total_trajectory.clear();

        // check the action sequence
        for(int i = 0; i < action_sequence.getActionSize(); i++)
        {
            // get the current motion task
            MotionTask m = action_sequence.getActionTaskAt(i);
            std::cout << "-------------------------- motion task " << i;
            if(m.is_in_manipulation_manifold)
                std::cout << " in manipulation manipulation manifold with id " << m.manifold_id << " of foliation " << m.foliation_id << std::endl;
            else
                std::cout << " in intermediate placement manifold with id " << m.manifold_id << " of foliation " << m.foliation_id << std::endl;

            // if the current task has solution, then continue.
            if(action_sequence.hasSolutionAt(i))
            {
                // reuse the solution from the task graph
                robot_trajectory::RobotTrajectory solution_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
                solution_trajectory.setRobotTrajectoryMsg(current_state, m.solution_trajectory);
                current_state = solution_trajectory.getLastWayPoint();

                // get the cartesian trajectory
                robot_trajectory::RobotTrajectory cartesian_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
                cartesian_trajectory.setRobotTrajectoryMsg(current_state, action_sequence.getCartesianMotionAt(i));

                // add the reused solution trajectory into the total trajectory.
                total_trajectory.append(solution_trajectory, 0.01);
                total_trajectory.append(cartesian_trajectory, 0.01);

                current_state = total_trajectory.getLastWayPoint();
                continue;
            }
        
            if(m.is_in_manipulation_manifold) // if current task is in manipulation manifold
            {
                // calculate current target object pose.
                Eigen::Isometry3d current_in_hand_pose;
                tf::transformTFToEigen(actual_grasp_transforms[m.manifold_id].inverse(), current_in_hand_pose);

                // 1. attach the object to the end effector in the planning scene.
                current_state.attachBody("target_object", current_in_hand_pose, target_object_shapes, shape_poses, std::vector<std::string>{"l_gripper_finger_link", "r_gripper_finger_link"}, "wrist_roll_link");
                
                // 2. set the proper planner in the move group
                move_group.setActionWithId("slide", m.manifold_id);

                // 3. set the constraint in this manifold
                move_group.setPathConstraints(manipulation_manifold_constraints[m.manifold_id]);
                move_group.setInHandPose(manipulation_manifold_constraints[m.manifold_id].in_hand_pose);

                // 4. set the start and target joint values
                move_group.setStartState(current_state);

                std::vector<double> target_joint_positions_double;
                for(int j = 0; j < m.target_joint_values.size(); j++)
                    target_joint_positions_double.push_back((double)m.target_joint_values[j]);
                move_group.setJointValueTarget(target_joint_positions_double);

                // 5. plan and execute the motion
                robot_trajectory::RobotTrajectory slide_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
                
                moveit::planning_interface::MoveGroupInterface::Plan slide_plan;
                std::vector<moveit::planning_interface::MoveGroupInterface::MotionEdge> experience;
                bool success = (move_group.plan(slide_plan, experience) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                
                // reset the move group
                move_group.clearPathConstraints();
                move_group.clearAction();
                move_group.clearExperience();
                move_group.clearPoseTargets();
                move_group.clearInHandPose();

                if(success)
                {
                    std::cout << "slide plan success" << std::endl;
                    slide_trajectory.setRobotTrajectoryMsg(current_state, slide_plan.trajectory_);
                    current_state = slide_trajectory.getLastWayPoint();

                    // set the solution in the action sequence
                    action_sequence.setSolutionForActionTaskAt(i, slide_plan.trajectory_);
                }
                else
                {
                    std::cout << "slide plan failed" << std::endl;
                    break;
                }

                // 6. execute the cartesian motion
                robot_trajectory::RobotTrajectory cartesian_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
                cartesian_trajectory.setRobotTrajectoryMsg(current_state, action_sequence.getCartesianMotionAt(i));

                // 7. add the re-grasp trajectory into the total trajectory.
                total_trajectory.append(slide_trajectory, 0.01);
                total_trajectory.append(cartesian_trajectory, 0.01);

                current_state = total_trajectory.getLastWayPoint();

                // 8. detach the object from the end effector in the planning scene.
                current_state.clearAttachedBody("target_object");
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
                robot_trajectory::RobotTrajectory regrasp_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
                
                moveit::planning_interface::MoveGroupInterface::Plan regrasp_plan;
                std::vector<moveit::planning_interface::MoveGroupInterface::MotionEdge> experience;
                bool success = (move_group.plan(regrasp_plan, experience) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if(success)
                {
                    std::cout << "regrasp plan success" << std::endl;
                    regrasp_trajectory.setRobotTrajectoryMsg(current_state, regrasp_plan.trajectory_);
                    current_state = regrasp_trajectory.getLastWayPoint();
                    // set the solution in the action sequence
                    action_sequence.setSolutionForActionTaskAt(i, regrasp_plan.trajectory_);
                }
                else
                {
                    std::cout << "regrasp plan failed" << std::endl;
                    break;
                }

                move_group.clearPathConstraints();
                move_group.clearInHandPose();
                move_group.clearAction();
                move_group.clearExperience();
                move_group.clearPoseTargets();
                move_group.clearInHandPose();


                // 6. execute the cartesian motion
                robot_trajectory::RobotTrajectory cartesian_trajectory = robot_trajectory::RobotTrajectory(kinematic_model, joint_model_group);
                cartesian_trajectory.setRobotTrajectoryMsg(current_state, action_sequence.getCartesianMotionAt(i));

                // 7. add the re-grasp trajectory into the total trajectory.
                total_trajectory.append(regrasp_trajectory, 0.01);
                total_trajectory.append(cartesian_trajectory, 0.01);

                current_state = total_trajectory.getLastWayPoint();

                // 8. remove the object and the table from the planning scene
                collision_object_target.operation = collision_object_target.REMOVE;
                planning_scene_interface.applyCollisionObject(collision_object_target);
                collision_object_table.operation = collision_object_table.REMOVE;
                planning_scene_interface.applyCollisionObject(collision_object_table);
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
            std::cout << "solution is not good, re-plan the action sequence" << std::endl;
            task_planner.updateTaskPlanner(action_sequence);
        }
        else
        {
            std::cout << "solution is good, break the loop" << std::endl;
            break;
        }
    }

    // visualize the trajectory
    trajectory_visuals->publishTrajectoryPath(total_trajectory);

    return 0;
    }
    */