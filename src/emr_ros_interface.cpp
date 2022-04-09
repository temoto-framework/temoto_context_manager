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

#include "temoto_context_manager/emr_ros_interface.h"
#include <boost/algorithm/string.hpp>

namespace emr_ros_interface
{
using namespace temoto_context_manager;
template <class Container>
void EmrRosInterface::publishContainerTf(const Container& container)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(container.pose.pose.position.x,
                                  container.pose.pose.position.y,
                                  container.pose.pose.position.z));
  transform.setRotation(tf::Quaternion(container.pose.pose.orientation.x,
                                      container.pose.pose.orientation.y,
                                      container.pose.pose.orientation.z,
                                      container.pose.pose.orientation.w));
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), temoto_core::common::toSnakeCase(container.parent), temoto_core::common::toSnakeCase(container.name)));
}

ObjectContainer EmrRosInterface::getObject(const std::string& name)
{
  return getContainer<ObjectContainer>(name);
}
MapContainer EmrRosInterface::getMap(const std::string& name)
{
  return getContainer<MapContainer>(name);
}
ComponentContainer EmrRosInterface::getComponent(const std::string& name)
{
  return getContainer<ComponentContainer>(name);
}
RobotContainer EmrRosInterface::getRobot(const std::string& name)
{
  return getContainer<RobotContainer>(name);
}
ObjectContainer EmrRosInterface::getNearestParentObject(const std::string& name)
{
  return getNearestParentOfType<ObjectContainer>(name);
}
MapContainer EmrRosInterface::getNearestParentMap(const std::string& name)
{
  return getNearestParentOfType<MapContainer>(name);
}
ComponentContainer EmrRosInterface::getNearestParentComponent(const std::string& name)
{
  return getNearestParentOfType<ComponentContainer>(name);
}
RobotContainer EmrRosInterface::getNearestParentRobot(const std::string& name)
{
  return getNearestParentOfType<RobotContainer>(name);
}

void EmrRosInterface::updatePose(const std::string& name, const geometry_msgs::PoseStamped& newPose)
{
  std::string type = getTypeByName(name);
  if (type == emr_ros_interface::emr_containers::OBJECT) 
  {
    updatePoseHelper<ObjectContainer>(name, newPose);
  }
  else if (type == emr_ros_interface::emr_containers::MAP)
  {
    updatePoseHelper<MapContainer>(name, newPose);
  }
  else if (type == emr_ros_interface::emr_containers::COMPONENT) 
  {
    updatePoseHelper<ComponentContainer>(name, newPose);
  }
  else if (type == emr_ros_interface::emr_containers::ROBOT) 
  {
    updatePoseHelper<RobotContainer>(name, newPose);
  }
}

std::string EmrRosInterface::getTypeByName(const std::string& name)
{
  return env_model_repository_.getItemByName(
      temoto_core::common::toSnakeCase(name))->getPayload()->getType();
}

std::vector<ItemContainer> EmrRosInterface::updateEmr(const ItemContainer & item_to_add, bool update_time)
{
  std::vector<temoto_context_manager::ItemContainer> items {item_to_add};
  return updateEmr(items, update_time);
}
bool EmrRosInterface::hasItem(const std::string& name) 
{
  return env_model_repository_.hasItem(temoto_core::common::toSnakeCase(name));
}
void EmrRosInterface::emrTfCallback(const ros::TimerEvent&)
{
  std::lock_guard<std::mutex> lock(emr_iface_mutex);
  for (auto const& item_entry : env_model_repository_.getItems())
  {
    // If root node, tf can not be published
    if (item_entry.second->getParent().expired()) continue;

    std::string type = item_entry.second->getPayload()->getType();
    if (type == emr_containers::OBJECT)
    {
      if (getRosPayloadPtr<temoto_context_manager::ObjectContainer>(item_entry.first)->getMaintainer() != identifier_) continue;
      temoto_context_manager::ObjectContainer oc = getContainerUnsafe<temoto_context_manager::ObjectContainer>(item_entry.first);
      publishContainerTf(oc);
    }
    else if (type == emr_containers::MAP)
    {
      if (getRosPayloadPtr<temoto_context_manager::MapContainer>(item_entry.first)->getMaintainer() != identifier_) continue;
      temoto_context_manager::MapContainer mc = getContainerUnsafe<temoto_context_manager::MapContainer>(item_entry.first);
      publishContainerTf(mc);
    }
    else if (type == emr_containers::COMPONENT)
    {
      if (getRosPayloadPtr<temoto_context_manager::ComponentContainer>(item_entry.first)->getMaintainer() != identifier_) continue;
      temoto_context_manager::ComponentContainer mc = getContainerUnsafe<temoto_context_manager::ComponentContainer>(item_entry.first);
      publishContainerTf(mc);
    }
    else if (type == emr_containers::ROBOT)
    {
      if (getRosPayloadPtr<temoto_context_manager::RobotContainer>(item_entry.first)->getMaintainer() != identifier_) continue;
      temoto_context_manager::RobotContainer mc = getContainerUnsafe<temoto_context_manager::RobotContainer>(item_entry.first);
      publishContainerTf(mc);
    }
  }
}

std::vector<temoto_context_manager::ItemContainer> EmrRosInterface::updateEmr(
                  const std::vector<temoto_context_manager::ItemContainer>& items_to_add, 
                  bool update_time)
{
  std::lock_guard<std::mutex> lock(emr_iface_mutex);
  
  // Keep track of failed add/update attempts
  std::vector<temoto_context_manager::ItemContainer> failed_items;
  for (const auto& item_container : items_to_add)
  {
    if (item_container.type == emr_containers::OBJECT) 
    {
      // Deserialize into an temoto_context_manager::ObjectContainer object and add to EMR
      temoto_context_manager::ObjectContainer oc = 
        temoto_core::deserializeROSmsg<temoto_context_manager::ObjectContainer>(item_container.serialized_container);
      if (!addOrUpdateEmrItem(oc, emr_containers::OBJECT, item_container.maintainer, update_time)) failed_items.push_back(item_container);
    }
    else if (item_container.type == emr_containers::MAP) 
    {
      // Deserialize into an temoto_context_manager::MapContainer object and add to EMR
      temoto_context_manager::MapContainer mc = 
        temoto_core::deserializeROSmsg<temoto_context_manager::MapContainer>(item_container.serialized_container);
      if (!addOrUpdateEmrItem(mc, emr_containers::MAP, item_container.maintainer, update_time)) failed_items.push_back(item_container);
    }
    else if (item_container.type == emr_containers::COMPONENT) 
    {
      // Deserialize into an temoto_context_manager::ComponentContainer object and add to EMR
      temoto_context_manager::ComponentContainer cc = 
        temoto_core::deserializeROSmsg<temoto_context_manager::ComponentContainer>(item_container.serialized_container);
      if (!addOrUpdateEmrItem(cc, emr_containers::COMPONENT, item_container.maintainer, update_time)) failed_items.push_back(item_container);
    }
    else if (item_container.type == emr_containers::ROBOT) 
    {
      // Deserialize into an temoto_context_manager::ComponentContainer object and add to EMR
      temoto_context_manager::RobotContainer cc = 
        temoto_core::deserializeROSmsg<temoto_context_manager::RobotContainer>(item_container.serialized_container);
      if (!addOrUpdateEmrItem(cc, emr_containers::ROBOT, item_container.maintainer, update_time)) failed_items.push_back(item_container);
    }
    else
    {
      ROS_ERROR_STREAM("Wrong type " << item_container.type.c_str() << "specified for EMR item");
      failed_items.push_back(item_container);
    }
  }

  return failed_items;
}

std::vector<temoto_context_manager::ItemContainer> EmrRosInterface::EmrToVector()
{
  std::lock_guard<std::mutex> lock(emr_iface_mutex);
  std::vector<temoto_context_manager::ItemContainer> items;
  std::vector<std::shared_ptr<emr::Item>> root_items = env_model_repository_.getRootItems();
  for (const auto& item : root_items)
  {
    EmrToVectorHelper(*item, items);
  }
  return items;
}

void EmrRosInterface::EmrToVectorHelper(const emr::Item& currentItem, std::vector<temoto_context_manager::ItemContainer>& items)
{
  // Create empty container and fill it based on the payload
  temoto_context_manager::ItemContainer ic;
  
  ic.type = currentItem.getPayload()->getType();

  // Get the item payload as ROS msg
  // To do this we dynamically cast the base class to the appropriate
  //   derived class and call the getPayload() method
  if (ic.type == emr_containers::OBJECT) 
  {
    std::shared_ptr<RosPayload<temoto_context_manager::ObjectContainer>> rospl = 
      std::dynamic_pointer_cast<RosPayload<temoto_context_manager::ObjectContainer>>(currentItem.getPayload());
    ic.serialized_container = temoto_core::serializeROSmsg(rospl->getPayload());
    ic.maintainer = rospl->getMaintainer();
    items.push_back(ic);

  }
  else if (ic.type == emr_containers::MAP) 
  {
    std::shared_ptr<RosPayload<temoto_context_manager::MapContainer>> rospl = 
      std::dynamic_pointer_cast<RosPayload<temoto_context_manager::MapContainer>>(currentItem.getPayload());
    ic.serialized_container = temoto_core::serializeROSmsg(rospl->getPayload());
    ic.maintainer = rospl->getMaintainer();
    items.push_back(ic);

  }
  else if (ic.type == emr_containers::COMPONENT) 
  {
    std::shared_ptr<RosPayload<temoto_context_manager::ComponentContainer>> rospl = 
      std::dynamic_pointer_cast<RosPayload<temoto_context_manager::ComponentContainer>>(currentItem.getPayload());
    ic.serialized_container = temoto_core::serializeROSmsg(rospl->getPayload());
    ic.maintainer = rospl->getMaintainer();
    items.push_back(ic);

  }
  else if (ic.type == emr_containers::ROBOT) 
  {
    std::shared_ptr<RosPayload<temoto_context_manager::RobotContainer>> rospl = 
      std::dynamic_pointer_cast<RosPayload<temoto_context_manager::RobotContainer>>(currentItem.getPayload());
    ic.serialized_container = temoto_core::serializeROSmsg(rospl->getPayload());
    ic.maintainer = rospl->getMaintainer();
    items.push_back(ic);

  }
  else
  {
    ROS_ERROR_STREAM("Wrong type of container @ EmrToVectorHelper: " << ic.type);
    return;
  }

  std::vector<std::shared_ptr<emr::Item>> children = currentItem.getChildren();
  for (uint32_t i = 0; i < children.size(); i++)
  {
    EmrToVectorHelper(*children[i], items);
  }

}
void EmrRosInterface::removeItem(const std::string& name)
{
  env_model_repository_.removeItem(name);
}
} // namespace emr_ros_interface