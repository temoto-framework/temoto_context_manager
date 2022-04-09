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
  {
    /*
     * Get the vector of ItemContainers and parse the ItemContainer to specific containers
     */
    ROS_INFO("Getting EMR items from the Context Manager and publishing them as rviz markers");

    temoto_context_manager::Items item_containers = cmi_.getEmrVector();
    for (auto& item_container : item_containers)
    {
      // Create a VisualizationMarker representation for each container
      visualization_msgs::Marker viz_marker_msg;
      bool viz_marker_defined = false;

      if (item_container.type == emr_ros_interface::emr_containers::OBJECT) 
      {
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
        viz_marker_defined = true;
      }
      else if (item_container.type == emr_ros_interface::emr_containers::MAP) 
      {
        // Deserialize into an temoto_context_manager::MapContainer object
        temoto_context_manager::MapContainer mc = 
          temoto_core::deserializeROSmsg<temoto_context_manager::MapContainer>(
            item_container.serialized_container);
      }
      else if (item_container.type == emr_ros_interface::emr_containers::COMPONENT) 
      {
        // Deserialize into an temoto_context_manager::ComponentContainer object
        temoto_context_manager::ComponentContainer cc = 
          temoto_core::deserializeROSmsg<temoto_context_manager::ComponentContainer>(
            item_container.serialized_container);
      }
      else if (item_container.type == emr_ros_interface::emr_containers::ROBOT) 
      {
        // Deserialize into an temoto_context_manager::RobotContainer object
        temoto_context_manager::RobotContainer cc = 
          temoto_core::deserializeROSmsg<temoto_context_manager::RobotContainer>(
            item_container.serialized_container);
        // Fill out the marker message details
        viz_marker_msg = cc.marker;
        viz_marker_msg.ns = cc.name;
        viz_marker_msg.header = cc.pose.header;
        viz_marker_msg.header.stamp = ros::Time::now();
        viz_marker_msg.pose = cc.pose.pose;
        viz_marker_defined = true;  
      }
      else
      {
        TEMOTO_ERROR_STREAM_("Wrong type " << item_container.type.c_str() << "specified for EMR item");
        continue;
      }

      // Publish the marker message
      if (viz_marker_publisher_ && viz_marker_defined)
      {
        viz_marker_publisher_.publish(viz_marker_msg);
      }
    }
    
    ros::Duration(3).sleep();
  }
  
  return 0;
}