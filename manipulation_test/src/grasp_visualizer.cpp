#include "ros/ros.h"
#include "manipulation_test/VisualizeGrasp.h"
#include "manipulation_test/VisualizeTable.h"
#include "manipulation_test/VisualizeObstacle.h"
#include "manipulation_test/VisualizeIntermediatePlacements.h"
#include "manipulation_test/VisualizeRegrasp.h"
#include <ros_tensorflow_msgs/ComPredict.h>
#include <tf/transform_listener.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

class GraspVisualizer
{
  public:
    GraspVisualizer(ros::NodeHandle &nh)
    {
      visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>("base_link", "/rviz_visual_tools");
      segmented_object_pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_object_for_com_pred", 1);
      segmented_table_pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_table_for_com_pred", 1);
      has_init = false;
      has_set_table = false;
    }

    // the grasp pose for visualization
    bool setGrasp(manipulation_test::VisualizeGrasp::Request &req,
	          manipulation_test::VisualizeGrasp::Response &res)
    {

      ROS_INFO("Receive set grasp request with object: %s", req.grasped_object_name.c_str());

      if(not listener.frameExists(req.grasped_object_name))
      {
        ROS_INFO("the object does not exist!");
        return true;
      }

      OBJECT_NAME = req.grasped_object_name;
      // load the grasp_data_list
      grasp_data_list.clear();
      for(int i = 0; i < req.grasp_poses.size(); i++)
      {
        GraspData grasp_data;
        grasp_data.grasp_transform.setOrigin(tf::Vector3(req.grasp_poses[i].pose.position.x,
                                                         req.grasp_poses[i].pose.position.y,
                                                         req.grasp_poses[i].pose.position.z));
        grasp_data.grasp_transform.setRotation(tf::Quaternion(req.grasp_poses[i].pose.orientation.x,
                                                              req.grasp_poses[i].pose.orientation.y,
                                                              req.grasp_poses[i].pose.orientation.z,
                                                              req.grasp_poses[i].pose.orientation.w));
        grasp_data.gripper_width = req.grasp_jawwidths[i];
        grasp_data.grasp_type = req.grasp_types[i];
        grasp_data_list.push_back(grasp_data);
      }
      has_init = true;

      tf::StampedTransform object_transform;
      res.actual_grasp_poses.clear();
      // get the object pose.
      try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/" + OBJECT_NAME, "/base_link", now, ros::Duration(1.0));
        listener.lookupTransform("/base_link","/" + OBJECT_NAME, now, object_transform);
      }
      catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      // find the actual grasp poses in the world frame
      for(int i = 0; i < grasp_data_list.size(); i++)
      {
        tf::Transform actual_grasp_transform = object_transform * grasp_data_list[i].grasp_transform;
        geometry_msgs::PoseStamped actual_grasp_pose;
        actual_grasp_pose.pose.position.x = actual_grasp_transform.getOrigin().x();
        actual_grasp_pose.pose.position.y = actual_grasp_transform.getOrigin().y();
        actual_grasp_pose.pose.position.z = actual_grasp_transform.getOrigin().z();
        actual_grasp_pose.pose.orientation.x = actual_grasp_transform.getRotation().x();
        actual_grasp_pose.pose.orientation.y = actual_grasp_transform.getRotation().y();
        actual_grasp_pose.pose.orientation.z = actual_grasp_transform.getRotation().z();
        actual_grasp_pose.pose.orientation.w = actual_grasp_transform.getRotation().w();

        res.actual_grasp_poses.push_back(actual_grasp_pose);
      }
      return true;
    }

    // the regrasp pose for visualize
    bool setRegrasp(manipulation_test::VisualizeRegrasp::Request &req,
                    manipulation_test::VisualizeRegrasp::Response &res)
    {
      ROS_INFO("Receive set regrasp request");
      regrasp_data_list.clear();
      for(int i = 0; i < req.grasp_poses.size(); i++)
      {
        GraspData regrasp_data;
        regrasp_data.grasp_transform.setOrigin(tf::Vector3(req.grasp_poses[i].pose.position.x,
                                                         req.grasp_poses[i].pose.position.y,
                                                         req.grasp_poses[i].pose.position.z));
        regrasp_data.grasp_transform.setRotation(tf::Quaternion(req.grasp_poses[i].pose.orientation.x,
                                                              req.grasp_poses[i].pose.orientation.y,
                                                              req.grasp_poses[i].pose.orientation.z,
                                                              req.grasp_poses[i].pose.orientation.w));
        regrasp_data.gripper_width = req.grasp_jawwidths[i];
        regrasp_data.grasp_type = req.grasp_types[i];
        regrasp_data_list.push_back(regrasp_data);
      }
      return true;
    }

    // the table pose for visualization
    bool setTable(manipulation_test::VisualizeTable::Request &req,
            manipulation_test::VisualizeTable::Response &res)
    {
      ROS_INFO("Receive set table request");
      tf::Transform table_transform;
      table_transform.setOrigin(tf::Vector3(req.center.x,
                                            req.center.y,
                                            req.center.z));
      table_transform.setRotation(tf::Quaternion(req.orientation.x,
                                                 req.orientation.y,
                                                 req.orientation.z,
                                                 req.orientation.w));

      transformTFToGeoPose(table_transform, table_pose);

      table_depth = req.depth;
      table_width = req.width;
      table_height = req.height;

      has_set_table = true;

      return true;
    }

    // the obstacle pose for visualization
    bool setObstacles(manipulation_test::VisualizeObstacle::Request &req,
                      manipulation_test::VisualizeObstacle::Response &res)
    {
      ROS_INFO("Receive set obstacle request");
      obstacle_poses.clear();
      obstacle_depths.clear();
      obstacle_widths.clear();
      obstacle_heights.clear();
      obstacle_text_poses.clear();
      obstacle_ids.clear();
      is_target.clear();
      for(int i = 0; i < req.obstacle_poses.size(); i++)
      {
        geometry_msgs::Pose obstacle_pose;
        obstacle_pose.position.x = req.obstacle_poses[i].pose.position.x;
        obstacle_pose.position.y = req.obstacle_poses[i].pose.position.y;
        obstacle_pose.position.z = req.obstacle_poses[i].pose.position.z;
        obstacle_pose.orientation.x = req.obstacle_poses[i].pose.orientation.x;
        obstacle_pose.orientation.y = req.obstacle_poses[i].pose.orientation.y;
        obstacle_pose.orientation.z = req.obstacle_poses[i].pose.orientation.z;
        obstacle_pose.orientation.w = req.obstacle_poses[i].pose.orientation.w;
        geometry_msgs::Pose obstacle_text_pose;
        obstacle_text_pose.position.x = req.obstacle_poses[i].pose.position.x;
        obstacle_text_pose.position.y = req.obstacle_poses[i].pose.position.y;
        obstacle_text_pose.position.z = req.obstacle_poses[i].pose.position.z + 0.2;
        obstacle_text_pose.orientation.x = req.obstacle_poses[i].pose.orientation.x;
        obstacle_text_pose.orientation.y = req.obstacle_poses[i].pose.orientation.y;
        obstacle_text_pose.orientation.z = req.obstacle_poses[i].pose.orientation.z;
        obstacle_text_pose.orientation.w = req.obstacle_poses[i].pose.orientation.w;
        obstacle_poses.push_back(obstacle_pose);
        obstacle_text_poses.push_back(obstacle_text_pose);
        obstacle_depths.push_back(req.depths[i]);
        obstacle_widths.push_back(req.widths[i]);
        obstacle_heights.push_back(req.heights[i]);
        obstacle_ids.push_back(req.obstacle_ids[i]);
        is_target.push_back(req.is_target[i]);
      }

      return true;
    }

    bool setIntermediatePlacements(manipulation_test::VisualizeIntermediatePlacements::Request &req,
                                   manipulation_test::VisualizeIntermediatePlacements::Response &res)
    {
      ROS_INFO("Receive set intermediate placements request");
      intermediate_placement_poses.clear();
      intermediate_placement_heights.clear();
      intermediate_placement_depths.clear();
      intermediate_placement_widths.clear();

      for(int i = 0; i < req.intermediate_placement_poses.size(); i++)
      {
        geometry_msgs::Pose intermediate_placement_pose;
        intermediate_placement_pose.position.x = req.intermediate_placement_poses[i].pose.position.x;
        intermediate_placement_pose.position.y = req.intermediate_placement_poses[i].pose.position.y;
        intermediate_placement_pose.position.z = req.intermediate_placement_poses[i].pose.position.z;
        intermediate_placement_pose.orientation.x = req.intermediate_placement_poses[i].pose.orientation.x;
        intermediate_placement_pose.orientation.y = req.intermediate_placement_poses[i].pose.orientation.y;
        intermediate_placement_pose.orientation.z = req.intermediate_placement_poses[i].pose.orientation.z;
        intermediate_placement_pose.orientation.w = req.intermediate_placement_poses[i].pose.orientation.w;
        intermediate_placement_poses.push_back(intermediate_placement_pose);
        intermediate_placement_heights.push_back(req.intermediate_placement_heights[i]);
        intermediate_placement_depths.push_back(req.intermediate_placement_depths[i]);
        intermediate_placement_widths.push_back(req.intermediate_placement_widths[i]);
      }

      return true;
    }

    bool setPointCloud(ros_tensorflow_msgs::ComPredict::Request &req, ros_tensorflow_msgs::ComPredict::Response &res)
    {
      ROS_INFO("Receive set point cloud request for com prediction");
      // point_cloud.clear();
      // for(int i = 0; i < req.point_cloud.size(); i++)
      // {
      //   point_cloud.push_back(req.point_cloud[i]);
      // }
      segmented_object_pc = req.segmented_point_cloud;
      segmented_table_pc = req.table_point_cloud;

      return true;
    }

    void update()
    {
      segmented_object_pub.publish(segmented_object_pc);
      segmented_table_pub.publish(segmented_table_pc);
      visual_tools_->deleteAllMarkers();

      for(int i = 0; i < regrasp_data_list.size(); i++)
      {
        visualizeGrasp(regrasp_data_list[i].grasp_transform, regrasp_data_list[i].gripper_width, regrasp_data_list[i].grasp_type == 0 ? rviz_visual_tools::GREEN : rviz_visual_tools::RED);
      }

      visualizeTable();

      visualizeObstacles();

      visualizeIntermediatePlacements();

      visual_tools_->trigger();
    }

    void visualizeObject(const tf::Transform &object_transform)
    {
      geometry_msgs::Pose object_pose;
      transformTFToGeoPose(object_transform, object_pose);
      visual_tools_->publishMesh(object_pose, "package://objects_description/" + OBJECT_NAME + "/" + OBJECT_NAME + ".stl", rviz_visual_tools::YELLOW, 1.0, OBJECT_NAME, 1);
    }

    void visualizeGrasp(const tf::Transform &gripper_transform, float gripper_width = 0.08, rviz_visual_tools::colors color = rviz_visual_tools::GREEN)
    {
      if(gripper_width > 0.08)
        gripper_width = 0.08;
      if(gripper_width < 0.0)
        gripper_width = 0.0;

      tf::Vector3 gripper_axis_1 = gripper_transform * tf::Vector3(0.13, gripper_width / 2, 0);
      tf::Vector3 gripper_fingertip_1 = gripper_transform * tf::Vector3(0.2, gripper_width / 2, 0);
      tf::Vector3 gripper_axis_2 = gripper_transform * tf::Vector3(0.13, -gripper_width / 2, 0);
      tf::Vector3 gripper_fingertip_2 = gripper_transform * tf::Vector3(0.2, -gripper_width / 2, 0);
      tf::Vector3 gripper_center = gripper_transform * tf::Vector3(0.13, 0, 0);
      tf::Vector3 gripper_end = gripper_transform * tf::Vector3(0.09, 0, 0);
      geometry_msgs::Point gripper_axis_1_point, gripper_axis_2_point, gripper_fingertip_1_point, gripper_fingertip_2_point, gripper_center_point, gripper_end_point;
      tf::pointTFToMsg(gripper_axis_1, gripper_axis_1_point);
      tf::pointTFToMsg(gripper_axis_2, gripper_axis_2_point);
      tf::pointTFToMsg(gripper_fingertip_1, gripper_fingertip_1_point);
      tf::pointTFToMsg(gripper_fingertip_2, gripper_fingertip_2_point);
      tf::pointTFToMsg(gripper_center, gripper_center_point);
      tf::pointTFToMsg(gripper_end, gripper_end_point);
      visual_tools_->publishLine(gripper_axis_1_point, gripper_axis_2_point, color, rviz_visual_tools::MEDIUM);
      visual_tools_->publishLine(gripper_axis_1_point, gripper_fingertip_1_point, color, rviz_visual_tools::MEDIUM);
      visual_tools_->publishLine(gripper_axis_2_point, gripper_fingertip_2_point, color, rviz_visual_tools::MEDIUM);
      visual_tools_->publishLine(gripper_center_point, gripper_end_point, color, rviz_visual_tools::MEDIUM);
    }

    void visualizeTable()
    {
      if(not has_set_table)
        return;
      visual_tools_->publishCuboid(table_pose, table_depth, table_width, table_height, rviz_visual_tools::BLUE);
    }

    void visualizeObstacles()
    {
      for(int i = 0; i < obstacle_poses.size(); i++)
      {
        if(is_target[i])
          visual_tools_->publishCuboid(obstacle_poses[i], obstacle_depths[i], obstacle_widths[i], obstacle_heights[i], rviz_visual_tools::GREEN);
        else
          visual_tools_->publishCuboid(obstacle_poses[i], obstacle_depths[i], obstacle_widths[i], obstacle_heights[i], rviz_visual_tools::RED);
        visual_tools_->publishText(obstacle_text_poses[i], "id " + std::to_string(obstacle_ids[i]), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE, false);
      }
    }

    void visualizeIntermediatePlacements()
    {
      for(int i = 0; i < intermediate_placement_poses.size(); i++)
      {
        visual_tools_->publishCuboid(intermediate_placement_poses[i], intermediate_placement_depths[i], intermediate_placement_widths[i], intermediate_placement_heights[i], rviz_visual_tools::YELLOW);
      }
    }

  private:
    struct GraspData
    {
      tf::Transform grasp_transform;
      float gripper_width;
      int grasp_type;
    };

    void transformTFToGeoPose(const tf::Transform &t, geometry_msgs::Pose &p){
      p.orientation.x = t.getRotation().x();
      p.orientation.y = t.getRotation().y();
      p.orientation.z = t.getRotation().z();
      p.orientation.w = t.getRotation().w();
      p.position.x = t.getOrigin().x();
      p.position.y = t.getOrigin().y();
      p.position.z = t.getOrigin().z();
    }

    ros::Publisher segmented_object_pub;
    ros::Publisher segmented_table_pub;


    tf::TransformListener listener;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    std::string OBJECT_NAME;
    std::vector<GraspData> grasp_data_list;
    std::vector<GraspData> regrasp_data_list;
    geometry_msgs::Pose table_pose;
    float table_depth, table_width, table_height;
    std::vector<geometry_msgs::Pose> obstacle_poses;
    std::vector<geometry_msgs::Pose> obstacle_text_poses;
    std::vector<geometry_msgs::Pose> intermediate_placement_poses;
    std::vector<float> obstacle_depths, obstacle_widths, obstacle_heights;
    std::vector<float> intermediate_placement_depths, intermediate_placement_widths, intermediate_placement_heights;
    std::vector<int> obstacle_ids;
    std::vector<bool> is_target;

    sensor_msgs::PointCloud2 segmented_object_pc;
    sensor_msgs::PointCloud2 segmented_table_pc;

    bool has_init;
    bool has_set_table;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_visulizer_server");
  ros::NodeHandle n;

  GraspVisualizer graspvis(n);

  ros::ServiceServer visualize_grasp_service = n.advertiseService("visualize_grasp", &GraspVisualizer::setGrasp, &graspvis);

  ros::ServiceServer visualize_regrasp_service = n.advertiseService("visualize_regrasp", &GraspVisualizer::setRegrasp, &graspvis);

  ros::ServiceServer visualize_table_service = n.advertiseService("visualize_table", &GraspVisualizer::setTable, &graspvis);

  ros::ServiceServer visualize_obstacle_service = n.advertiseService("visualize_obstacle", &GraspVisualizer::setObstacles, &graspvis);

  ros::ServiceServer visualize_intermediate_placements_service = n.advertiseService("visualize_intermediate_placements", &GraspVisualizer::setIntermediatePlacements, &graspvis);

  ros::ServiceServer visualize_point_cloud_service = n.advertiseService("visualize_point_cloud", &GraspVisualizer::setPointCloud, &graspvis);

  ros::Rate r(5);
  while(ros::ok())
  {
    graspvis.update();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
