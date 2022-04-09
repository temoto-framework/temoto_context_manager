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

/* Author: Meelis Pihlap */

#ifndef TEMOTO_CONTEXT_MANAGER__EMR_ROS_INTERFACE_H
#define TEMOTO_CONTEXT_MANAGER__EMR_ROS_INTERFACE_H

#include <algorithm>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include "ros/package.h"

#include "temoto_context_manager/context_manager_containers.h"
#include "temoto_context_manager/env_model_repository.h"
#include "temoto_context_manager/env_model_interface.h"
#include "temoto_core/common/ros_serialization.h"
#include "temoto_core/common/tools.h"
#include "geometry_msgs/PoseStamped.h"

namespace emr_ros_interface
{
using namespace temoto_context_manager;

/**
 * @brief EMR payload that houses ROS messages
 * 
 * @tparam RosMsg 
 */
template <class RosMsg>
class RosPayload : public emr::PayloadEntry
{
private:
  RosMsg payload_;
  std::string maintainer_;
public:
  /**
   * @brief updates timestamp of stored message to now
   * 
   */
  void updateTime()
  {
    payload_.pose.header.stamp = ros::Time::now();
  }
  /**
   * @brief Updates time of stored message
   * 
   * @param new_time 
   */
  void updateTime(ros::Time new_time)
  {
    payload_.pose.header.stamp = new_time;
  }
  /**
   * @brief Returns the message timestamp
   * 
   * @return ros::Time 
   */
  ros::Time getTime() {return payload_.pose.header.stamp;}
  /**
   * @brief Return the name of maintainer.
   * 
   * The maintainer is responsible for publishing ROS transforms between this item and parent
   * 
   * @return std::string 
   */
  std::string getMaintainer() {return maintainer_;}
  /**
   * @brief Set the maintainer 
   * 
   * @param maintainer 
   */
  void setMaintainer(const std::string& maintainer)
  {
    maintainer_ = maintainer;
  }
  /**
   * @brief Get the name of this item
   * 
   * @return std::string 
   */
  const std::string& getName() const
  {
    return payload_.name;
  }
  /**
   * @brief Get the payload 
   * 
   * @return RosMsg 
   */
  RosMsg getPayload() const {return payload_;};
  /**
   * @brief Set the payload 
   * 
   * @param payload 
   */
  void setPayload(RosMsg & payload) {payload_ = payload;};
  
  RosPayload(RosMsg payload) : payload_(payload)
  {
  }
  RosPayload(RosMsg payload, std::string maintainer) 
    : payload_(payload), maintainer_(maintainer)
  {
  }

};

class EmrRosInterface : public temoto_context_manager::EnvModelInterface
{
public:

  // Inherited functions, for more info see definition of EnvModelInterface

  std::string getTypeByName(const std::string& name);

  // C++ does not support templated virtual classes :(
  temoto_context_manager::ObjectContainer getObject(const std::string& name);
  temoto_context_manager::MapContainer getMap(const std::string& name);
  temoto_context_manager::ComponentContainer getComponent(const std::string& name);
  temoto_context_manager::RobotContainer getRobot(const std::string& name);

  temoto_context_manager::ObjectContainer getNearestParentObject(const std::string& name);
  temoto_context_manager::MapContainer getNearestParentMap(const std::string& name);
  temoto_context_manager::ComponentContainer getNearestParentComponent(const std::string& name);
  temoto_context_manager::RobotContainer getNearestParentRobot(const std::string& name);
  void removeItem(const std::string& name);
  bool hasItem(const std::string& name);
  std::vector<ItemContainer> updateEmr(const std::vector<ItemContainer> & items_to_add, bool update_time=false);
  std::vector<ItemContainer> updateEmr(const ItemContainer & item_to_add, bool update_time=false);
  std::vector<ItemContainer> EmrToVector();

  EmrRosInterface(emr::EnvironmentModelRepository& emr, std::string identifier) : env_model_repository_(emr), identifier_(identifier) 
  {
    // TODO: Move this to context manager
    tf_timer_ = nh_.createTimer(ros::Duration(0.1), &EmrRosInterface::emrTfCallback, this);
  }

  void updatePose(const std::string& name, const geometry_msgs::PoseStamped& newPose);

  /**
   * @brief Helper function to handle templates
   * 
   * @tparam Container 
   * @param name 
   * @param newPose 
   */
  template <class Container> 
  void updatePoseHelper(const std::string& name, const geometry_msgs::PoseStamped& newPose)
  {
    std::shared_ptr<RosPayload<Container>> plptr = getRosPayloadPtr<Container>(name); 
    std::lock_guard<std::mutex> lock(emr_iface_mutex);
    auto temp = plptr->getPayload();
    temp.pose = newPose;
    plptr->setPayload(temp);
  }

  /**
   * @brief Get the Container by name
   * 
   * @tparam Container 
   * @param name 
   * @return Container 
   */
  template<class Container>
  Container getContainer(const std::string& name)
  {
    std::lock_guard<std::mutex> lock(emr_iface_mutex);
    // TODO: What if a null pointer is returned?
    return getRosPayloadPtr<Container>(name)->getPayload();
  }

  /**
   * @brief Get the Container without locking mutex
   * 
   * TODO: Once tf handling is moved to the context manager, this method is redundant.
   * 
   * @tparam Container 
   * @param name 
   * @return Container 
   */
  template<class Container>
  Container getContainerUnsafe(const std::string& name)
  {
    return getRosPayloadPtr<Container>(name)->getPayload();
  }
  
  /**
   * @brief Get RosPayload pointer
   * 
   * @tparam Container 
   * @param name 
   * @return std::shared_ptr<RosPayload<Container>> 
   */
  template<class Container>
  std::shared_ptr<RosPayload<Container>> getRosPayloadPtr(const std::string& name)
  {
    if (!hasItem(temoto_core::common::toSnakeCase(name))) 
    ROS_ERROR_STREAM("NO ITEM " << temoto_core::common::toSnakeCase(name) << " FOUND");
    return std::dynamic_pointer_cast<RosPayload<Container>>
      (env_model_repository_.getItemByName(temoto_core::common::toSnakeCase(name))->getPayload());
  }

  /**
   * @brief Add or update a single item of the EMR
   * 
   * @tparam Container 
   * @param container 
   * @param container_type 
   */
  template <class Container>
  bool addOrUpdateEmrItem(const Container & container, 
                          const std::string& container_type, 
                          const std::string& maintainer, 
                          const bool update_time)
  {
  RosPayload<Container> rospl = RosPayload<Container>(container);
  rospl.setType(container_type);
  std::string name = temoto_core::common::toSnakeCase(container.name);
  std::string parent = temoto_core::common::toSnakeCase(container.parent);

  // Check for empty name field
  // Move these to the context manager interface maybe? TBD
  if (name == "") 
  {
    ROS_ERROR_STREAM("Empty string not allowed as EMR item name!");
    return false;
  }
  // Check if the parent exists
  if ((!parent.empty()) && (!env_model_repository_.hasItem(parent))) 
  {
    ROS_ERROR_STREAM("No parent with name " << parent << " found in EMR!");
    return false;
  }
  
  // Check if the object has to be added or updated
  if (!env_model_repository_.hasItem(name))
  {
    // Add the new item
    // TODO: resolve tf_prefixes, if type == component or robot, prepend maintainer
    rospl.setMaintainer(maintainer);
    std::shared_ptr<RosPayload<Container>> plptr = std::make_shared<RosPayload<Container>>(rospl);
    env_model_repository_.addItem(name, parent, plptr);
  }
  else
  {
    if (rospl.getTime() > getRosPayloadPtr<Container>(name)->getTime()) 
    {
      // Update the item information
      if (update_time) rospl.updateTime();
      std::shared_ptr<RosPayload<Container>> plptr = std::make_shared<RosPayload<Container>>(rospl);
      env_model_repository_.updateItem(name, plptr);
      ROS_INFO_STREAM("Updated item: " << name);
    }
  }
  return true;
}

  /**
   * @brief Recursive helper function to save EMR state
   * 
   * @param currentItem 
   * @param items 
   */
  void EmrToVectorHelper(const emr::Item& currentItem, std::vector<temoto_context_manager::ItemContainer>& items);

  /**
   * @brief Update pose of EMR item
   * 
   * @tparam Container 
   * @param name 
   * @param newPose 
   */
  
template <class Container>
Container getNearestParentOfType(const std::string& name)
{
  std::shared_ptr<emr::Item> itemptr = env_model_repository_.getItemByName(temoto_core::common::toSnakeCase(name));
  if (itemptr->isRoot()) 
    ROS_ERROR_STREAM("ROOT ITEM HAS NO PARENTS.");
  std::string nearest = 
          getNearestParentHelper(parseContainerType<Container>(), itemptr->getParent().lock());
  return getContainer<Container>(nearest);
}

std::string getNearestParentHelper(const std::string& type, const std::shared_ptr<emr::Item>& itemptr)
{
  if (itemptr->getPayload()->getType() == type) 
  {
    return itemptr->getPayload()->getName();
  }
  else
  {
    // Check if there is a parent
    if (itemptr->isRoot()) 
    {
      ROS_ERROR_STREAM("No parent item of type" << type << "found in EMR!");
      return "";
    }
    return getNearestParentHelper(type, itemptr->getParent().lock());
  }
}

private:
  emr::EnvironmentModelRepository& env_model_repository_;
  std::string identifier_;
  ros::NodeHandle nh_;
  ros::Timer tf_timer_;
  tf::TransformBroadcaster tf_broadcaster;
  mutable std::mutex emr_iface_mutex;
  
  void emrTfCallback(const ros::TimerEvent&);
  template <class Container>
  void publishContainerTf(const Container& container);
  template <class Container>
std::string parseContainerType()
{
  if (std::is_same<Container, temoto_context_manager::ObjectContainer>::value) 
  {
    return emr_containers::OBJECT;
  }
  else if (std::is_same<Container, temoto_context_manager::MapContainer>::value) 
  {
    return emr_containers::MAP;
  }
  else if (std::is_same<Container, temoto_context_manager::ComponentContainer>::value) 
  {
    return emr_containers::COMPONENT;
  }
  else if (std::is_same<Container, temoto_context_manager::RobotContainer>::value) 
  {
    return emr_containers::ROBOT;
  }
  ROS_ERROR_STREAM("UNRECOGNIZED TYPE");
  return "FAULTY_TYPE";
}

  /**
   * @brief Moves up the tree, returning the closest container of type Container
   * 
   * @tparam Container 
   */
  
  
};

} // namespace emr_ros_interface

#endif