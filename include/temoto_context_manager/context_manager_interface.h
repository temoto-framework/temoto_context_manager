/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_INTERFACE_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_INTERFACE_H

#include "temoto_context_manager/context_manager_services.h"
#include "temoto_context_manager/context_manager_containers.h"
#include "temoto_core/common/ros_serialization.h"
#include "temoto_core/common/tools.h"
#include "temoto_resource_registrar/temoto_logging.h"
#include "temoto_resource_registrar/temoto_error.h"
#include <vector>

namespace temoto_context_manager
{

class ContextManagerInterface
{
public:

  ContextManagerInterface()
  {
    // Add EMR service client
    update_EMR_client_ = nh_.serviceClient<UpdateEmr>(srv_name::SERVER_UPDATE_EMR);
    get_emr_item_client_ = nh_.serviceClient<GetEMRItem>(srv_name::SERVER_GET_EMR_ITEM);
    get_emr_vector_client_ = nh_.serviceClient<GetEMRVector>(srv_name::SERVER_GET_EMR_VECTOR);
  }

  std::vector<ItemContainer> getEmrVector()
  {
    GetEMRVector srv_msg;
    if (!get_emr_vector_client_.call<GetEMRVector>(srv_msg)) 
    {
      throw TEMOTO_ERRSTACK("Failed to call the server: '" + srv_name::SERVER_GET_EMR_VECTOR + "'");
    }
    return srv_msg.response.items;
  }
  /**
   * @brief Get a container from the EMR
   * 
   * @tparam Container 
   * @param name 
   * @return Container 
   */
  template<class Container>
  Container getEMRContainer(std::string name)
  {
    Container container;
    if (name == "") 
    {
      throw TEMOTO_ERRSTACK("The container is missing a name");
    }
    else
    {
      GetEMRItem srv_msg;
      srv_msg.request.name = name;
      if (std::is_same<Container, ObjectContainer>::value) 
      {
        srv_msg.request.type = "OBJECT";
      }
      else if (std::is_same<Container, MapContainer>::value) 
      {
        srv_msg.request.type = "MAP";
      }
      if (!get_emr_item_client_.call<GetEMRItem>(srv_msg)) 
      {
        throw TEMOTO_ERRSTACK("Failed to call the server: '" + srv_name::SERVER_GET_EMR_ITEM + "'");
      }

      TEMOTO_INFO_("Got a response! ");
      if (srv_msg.response.success) 
      {
        container = temoto_core::deserializeROSmsg<Container>(
                                  srv_msg.response.item.serialized_container);
        TEMOTO_INFO_("Got a response! ");
      }
      else
      {
        TEMOTO_ERROR_STREAM_("Invalid EMR type requested for item.");
      }
    }
    return container;
    
  }

  /**
   * @brief Add single container to EMR
   * 
   * @tparam Container 
   * @param container 
   */
  template <class Container>
  void addToEmr(const Container& container)
  {
    std::vector<Container> containers;
    containers.push_back(container);
    addToEmr(containers);
  }

  /**
   * @brief Add several containers to EMR
   * 
   * Containers must be predefined as ROS messages in the context manager.
   * A container can be any ROS message with a name, parent(optional) and a payload.
   * 
   * @tparam Container 
   * @param containers 
   */
  template <class Container>
  void addToEmr(std::vector<Container> & containers)
  {
    std::vector<temoto_context_manager::ItemContainer> item_containers;
    for (auto& container : containers)
    {
      if (container.name == "")
      {
        throw TEMOTO_ERRSTACK("The container is missing a name");
      }
      else
      {
        container.pose.header.stamp = ros::Time::now(); 
        temoto_context_manager::ItemContainer nc;
        // nc.last_modified = ros::Time::now();
        nc.maintainer = temoto_core::common::getTemotoNamespace();
        // Check the type of the container
        if (std::is_same<Container, ObjectContainer>::value) 
        {
          nc.type = emr_ros_interface::emr_containers::OBJECT;
        }
        else if (std::is_same<Container, MapContainer>::value) 
        {
          nc.type = emr_ros_interface::emr_containers::MAP;
        }
        else if (std::is_same<Container, ComponentContainer>::value) 
        {
          nc.type = emr_ros_interface::emr_containers::COMPONENT;
        }
        else if (std::is_same<Container, RobotContainer>::value) 
        {
          nc.type = emr_ros_interface::emr_containers::ROBOT;
        }
        nc.serialized_container = temoto_core::serializeROSmsg(container);
        item_containers.push_back(nc);
      }
    }
    UpdateEmr update_EMR_srvmsg;
    update_EMR_srvmsg.request.items = item_containers;

    TEMOTO_INFO_STREAM_("Calling " << srv_name::SERVER_UPDATE_EMR << " server ...");
    if (!update_EMR_client_.call<UpdateEmr>(update_EMR_srvmsg)) 
    {
      throw TEMOTO_ERRSTACK("Failed to call the server: '" + srv_name::SERVER_UPDATE_EMR + "'");
    }

    TEMOTO_INFO_STREAM_("Call to " << srv_name::SERVER_UPDATE_EMR << " was successful");
    for (auto item_container : update_EMR_srvmsg.response.failed_items)
    {
      if (item_container.type == emr_ros_interface::emr_containers::OBJECT) {
        auto container = temoto_core::deserializeROSmsg<ObjectContainer>
                                (item_container.serialized_container);
        TEMOTO_INFO_STREAM_("Failed to add item: " << container.name << std::endl);
      }
      else if (item_container.type == emr_ros_interface::emr_containers::MAP)
      {
        auto container = temoto_core::deserializeROSmsg<ObjectContainer>
                                (item_container.serialized_container);
        TEMOTO_INFO_STREAM_("Failed to add item: " << container.name << std::endl);
      }
      else if (item_container.type == emr_ros_interface::emr_containers::COMPONENT)
      {
        auto container = temoto_core::deserializeROSmsg<ComponentContainer>
                                (item_container.serialized_container);
        TEMOTO_INFO_STREAM_("Failed to add item: " << container.name << std::endl);
      }
      else if (item_container.type == emr_ros_interface::emr_containers::ROBOT)
      {
        auto container = temoto_core::deserializeROSmsg<RobotContainer>
                                (item_container.serialized_container);
        TEMOTO_INFO_STREAM_("Failed to add item: " << container.name << std::endl);
      }
    }
  }

  ~ContextManagerInterface(){}

private:

  std::string name_; 
  ros::NodeHandle nh_;
  ros::ServiceClient update_EMR_client_;
  ros::ServiceClient get_emr_item_client_;
  ros::ServiceClient get_emr_vector_client_;

};

/**
 * @brief Creates a visualization marker message out of basic parameters
 * @return visualization_msgs::Marker 
 */
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

/**
 * @brief Creates a pose message
 * @return geometry_msgs::Pose 
 */
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

} // namespace
#endif
