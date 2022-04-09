#include "ros/ros.h"
#include "temoto_context_manager/context_manager_interface.h"

visualization_msgs::Marker createMarker( std::string name 
, int type
, float scale_x
, float scale_y
, float scale_z
, float color_r
, float color_g
, float color_b)
{
  visualization_msgs::Marker viz_marker_msg;
  viz_marker_msg.ns = name;
  viz_marker_msg.id = 0;
  viz_marker_msg.lifetime = ros::Duration();
  viz_marker_msg.type = type;
  viz_marker_msg.action = visualization_msgs::Marker::ADD;

  viz_marker_msg.scale.x = scale_x;
  viz_marker_msg.scale.y = scale_y;
  viz_marker_msg.scale.z = scale_z;

  viz_marker_msg.color.r = color_r;
  viz_marker_msg.color.g = color_g;
  viz_marker_msg.color.b = color_b;
  viz_marker_msg.color.a = 1.0;

  return viz_marker_msg;
}

geometry_msgs::Pose createPose( float p_x
, float p_y
, float p_z
, float o_x
, float o_y
, float o_z
, float o_w)
{
  geometry_msgs::Pose pose;
  pose.position.x = p_x;
  pose.position.y = p_y;
  pose.position.z = p_z;
  pose.orientation.x = o_x;
  pose.orientation.y = o_y;
  pose.orientation.z = o_z;
  pose.orientation.w = o_w;
  return pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emr_item_setter");

  /*
   * Create Context Manager Interface object that provides a simplified
   * API for communicating with the Context Manager. 
   */
  temoto_context_manager::ContextManagerInterface cmi_;

  /*
   * Define a "workshop_map" and pass it to the Context Manager
   */
  ROS_INFO("Adding a workshop_map to Context Manager");
  temoto_context_manager::MapContainer workshop_map;
  workshop_map.name = "workshop_map";
  cmi_.addToEmr(workshop_map);

  /*
   * Define a "table", assign the "workshop_map" as its parent and pass it to the Context Manager
   */
  ROS_INFO("Adding a table to Context Manager");
  temoto_context_manager::ObjectContainer table;
  table.name = "table";
  table.parent = workshop_map.name;
  table.pose.header.frame_id = workshop_map.name;
  table.pose.pose = createPose(0, 0, 0.45, 0, 0, 0, 1);
  table.marker = createMarker(table.name, visualization_msgs::Marker::CUBE, 1.0, 2.0, 0.9, 0, 1, 0);
  cmi_.addToEmr(table);

  /*
   * Define a "bottle", assign the "table" as its parent and pass it to the Context Manager
   */
  ROS_INFO("Adding a bottle to Context Manager");
  temoto_context_manager::ObjectContainer bottle;
  bottle.name = "bottle";
  bottle.parent = table.name;
  bottle.pose.header.frame_id = table.name;
  bottle.pose.pose = createPose(0.0, 0.5, 0.55, 0, 0, 0, 1);
  bottle.marker = createMarker(bottle.name, visualization_msgs::Marker::CYLINDER, 0.1, 0.1, 0.2, 1, 0, 0);
  cmi_.addToEmr(bottle);

  /*
   * Define a "chair", assign the "workshop_map" as its parent and pass it to the Context Manager
   */
  ROS_INFO("Adding a chair to Context Manager");
  temoto_context_manager::ObjectContainer chair;
  chair.name = "chair";
  chair.parent = workshop_map.name;
  chair.pose.header.frame_id = workshop_map.name;
  chair.pose.pose = createPose(1, 0, 0.3, 0, 0, 0, 1); 
  chair.marker = createMarker(chair.name, visualization_msgs::Marker::CYLINDER, 0.4, 0.4, 0.6, 0, 0, 1);
  cmi_.addToEmr(chair);
  
  return 0;
}