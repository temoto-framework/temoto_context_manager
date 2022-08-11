/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * This example shows how Context Manager Interface can be used to fetch EMR items from the
 * Environment Model Repository that is hosted inside the Context Manager.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ros/ros.h"
#include "temoto_context_manager/context_manager_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emr_item_marker_publisher");

  /*
   * Create Context Manager Interface object that provides a simplified
   * API for communicating with the Context Manager. 
   */
  temoto_context_manager::ContextManagerInterface cmi_;

  // Nodehandle for subscribers and publishers
  ros::NodeHandle nh_;

  // Publisher for the ItemContainer items
  ros::Publisher viz_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("emr_item_markers", 10);

  // Start publishing the emr items
  while (ros::ok())
  try
  {
    /*
     * Get the vector of ItemContainers and parse the ItemContainer to specific containers
     */
    temoto_context_manager::Items item_containers = cmi_.getEmrVector();
    
    if (!item_containers.empty())
    {
      ROS_INFO("Publishing EMR items as visualization markers");
    }

    for (auto& item_container : item_containers)
    {
      // This example is designed to visualize only "OBJECT" EMR container
      if (item_container.type != emr_ros_interface::emr_containers::OBJECT) 
      {
        continue;
      }

      // Create a VisualizationMarker representation for each container
      visualization_msgs::Marker viz_marker_msg;

      // Deserialize into an temoto_context_manager::ObjectContainer object
      temoto_context_manager::ObjectContainer oc = 
        temoto_core::deserializeROSmsg<temoto_context_manager::ObjectContainer>(
          item_container.serialized_container);

      // Fill out the marker message details
      viz_marker_msg = oc.marker;
      viz_marker_msg.ns = oc.name;
      viz_marker_msg.header = oc.pose.header;
      viz_marker_msg.header.stamp = ros::Time::now();
      viz_marker_msg.pose = oc.pose.pose;
      viz_marker_msg.lifetime = ros::Duration(3.0);
      viz_marker_publisher_.publish(viz_marker_msg);
    }
    
    ros::Duration(3).sleep();
  }
  catch (resource_registrar::TemotoErrorStack e)
  {
    ROS_WARN("Seems like the Context Manager is not up yet. Waiting a bit ...");
    ros::Duration(3).sleep();
  }

  return 0;
}