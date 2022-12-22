#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include "manipulation_test/VisualizeGrasp.h"
#include "manipulation_test/VisualizeTable.h"
#include "manipulation_test/VisualizeObstacle.h"
#include "manipulation_test/VisualizeIntermediatePlacements.h"

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

bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(), 
        s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_pipeline");

    ros::NodeHandle node_handle;

    // visualize the grasp poses over the current object
    ros::ServiceClient client = node_handle.serviceClient<manipulation_test::VisualizeGrasp>("visualize_grasp");
    client.waitForExistence();

    manipulation_test::VisualizeGrasp srv;
    srv.request.grasped_object_name = "hammer";

    // set the lifting grasp pose
    geometry_msgs::PoseStamped grasp_pose1;
    grasp_pose1.pose.position.x = 0;
    grasp_pose1.pose.position.y = 0;
    grasp_pose1.pose.position.z = 0.2;
    grasp_pose1.pose.orientation.x = -0.5;
    grasp_pose1.pose.orientation.y = -0.5;
    grasp_pose1.pose.orientation.z = 0.5;
    grasp_pose1.pose.orientation.w = -0.5;

    srv.request.grasp_poses.push_back(grasp_pose1);
    srv.request.grasp_jawwidths.push_back(0.08);
    srv.request.grasp_types.push_back(0);

    // set the sliding grasp pose
    geometry_msgs::PoseStamped grasp_pose2;
    grasp_pose2.pose.position.x = 0;
    grasp_pose2.pose.position.y = 0.12;
    grasp_pose2.pose.position.z = 0.2;
    grasp_pose2.pose.orientation.x = -0.5;
    grasp_pose2.pose.orientation.y = -0.5;
    grasp_pose2.pose.orientation.z = 0.5;
    grasp_pose2.pose.orientation.w = -0.5;

    srv.request.grasp_poses.push_back(grasp_pose2);
    srv.request.grasp_jawwidths.push_back(0.08);
    srv.request.grasp_types.push_back(1);

    // set another sliding grasp pose
    geometry_msgs::PoseStamped grasp_pose3;
    grasp_pose3.pose.position.x = 0;
    grasp_pose3.pose.position.y = 0.25;
    grasp_pose3.pose.position.z = 0.2;
    grasp_pose3.pose.orientation.x = -0.5;
    grasp_pose3.pose.orientation.y = -0.5;
    grasp_pose3.pose.orientation.z = 0.5;
    grasp_pose3.pose.orientation.w = -0.5;

    srv.request.grasp_poses.push_back(grasp_pose3);
    srv.request.grasp_jawwidths.push_back(0.08);
    srv.request.grasp_types.push_back(1);

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

    // get the target object transform in the table frame
    tf::Transform target_object_transform_in_table_frame = table_transform.inverse() * target_object_transform;

    // get 20 random target object pose in the table frame.
    std::vector<tf::Transform> random_target_object_transforms;
    for(int i = 0; i < 20; i++)
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

    // visualize the random target object poses
    ros::ServiceClient intermediate_placements_visualizer = node_handle.serviceClient<manipulation_test::VisualizeIntermediatePlacements>("visualize_intermediate_placements");
    intermediate_placements_visualizer.waitForExistence();

    manipulation_test::VisualizeIntermediatePlacements intermediate_placements_visualize_srv;
    for(int i = 0; i < random_target_object_transforms.size(); i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = random_target_object_transforms[i].getOrigin().x();
        pose_stamped.pose.position.y = random_target_object_transforms[i].getOrigin().y();
        pose_stamped.pose.position.z = random_target_object_transforms[i].getOrigin().z();
        pose_stamped.pose.orientation.x = random_target_object_transforms[i].getRotation().x();
        pose_stamped.pose.orientation.y = random_target_object_transforms[i].getRotation().y();
        pose_stamped.pose.orientation.z = random_target_object_transforms[i].getRotation().z();
        pose_stamped.pose.orientation.w = random_target_object_transforms[i].getRotation().w();
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