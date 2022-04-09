/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * This example shows how Context Manager Interface can be used to add new items to the
 * Environment Model Repository that is hosted inside the Context Manager.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros/ros.h"
#include "temoto_context_manager/context_manager_interface.h"

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
   * Define a "table", assign the "workshop_map" as its parent and pass it to the Context Manager.
   * OPTIONAL: visualization_msgs::Marker based shape is added to the item.
   */
  ROS_INFO("Adding a table to Context Manager");
  temoto_context_manager::ObjectContainer table;
  table.name = "table";
  table.parent = workshop_map.name;
  table.pose.header.frame_id = workshop_map.name;
  table.pose.pose = temoto_context_manager::createPose(0, 0, 0.45, 0, 0, 0, 1);
  table.marker = temoto_context_manager::createMarker(table.name, visualization_msgs::Marker::CUBE, 1.0, 2.0, 0.9, 0, 1, 0);
  cmi_.addToEmr(table);

  /*
   * Define a "bottle", assign the "table" as its parent and pass it to the Context Manager.
   * OPTIONAL: visualization_msgs::Marker based shape is added to the item.
   */
  ROS_INFO("Adding a bottle to Context Manager");
  temoto_context_manager::ObjectContainer bottle;
  bottle.name = "bottle";
  bottle.parent = table.name;
  bottle.pose.header.frame_id = table.name;
  bottle.pose.pose = temoto_context_manager::createPose(0.0, 0.5, 0.55, 0, 0, 0, 1);
  bottle.marker = temoto_context_manager::createMarker(bottle.name, visualization_msgs::Marker::CYLINDER, 0.1, 0.1, 0.2, 1, 0, 0);
  cmi_.addToEmr(bottle);

  /*
   * Define a "chair", assign the "workshop_map" as its parent and pass it to the Context Manager.
   * OPTIONAL: visualization_msgs::Marker based shape is added to the item.
   */
  ROS_INFO("Adding a chair to Context Manager");
  temoto_context_manager::ObjectContainer chair;
  chair.name = "chair";
  chair.parent = workshop_map.name;
  chair.pose.header.frame_id = workshop_map.name;
  chair.pose.pose = temoto_context_manager::createPose(1, 0, 0.3, 0, 0, 0, 1); 
  chair.marker = temoto_context_manager::createMarker(chair.name, visualization_msgs::Marker::CYLINDER, 0.4, 0.4, 0.6, 0, 0, 1);
  cmi_.addToEmr(chair);
  
  return 0;
}