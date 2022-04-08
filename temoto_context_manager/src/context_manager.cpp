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

#include "ros/package.h"
#include "temoto_core/common/ros_serialization.h"
#include "temoto_context_manager/context_manager.h"
#include "temoto_resource_registrar/temoto_logging.h"
#include "temoto_resource_registrar/temoto_error.h"
#include <algorithm>
#include <utility>

namespace temoto_context_manager
{

ContextManager::ContextManager()
: temoto_core::BaseSubsystem(srv_name::MANAGER, temoto_core::error::Subsystem::COMPONENT_MANAGER, __func__)
, emr_syncer_(srv_name::MANAGER, srv_name::SYNC_OBJECTS_TOPIC, &ContextManager::emrSyncCb, this)
, resource_registrar_(srv_name::MANAGER)
{
  /*
   * Initialize the Environment Model Repository interface
   */
  emr_interface = std::make_shared<emr_ros_interface::EmrRosInterface>(env_model_repository_, temoto_core::common::getTemotoNamespace());

  // "Update EMR" 
  TEMOTO_INFO_("Starting the EMR update server");
  update_emr_server_ = nh_.advertiseService(srv_name::SERVER_UPDATE_EMR, &ContextManager::updateEmrCb, this);
  get_emr_item_server_ = nh_.advertiseService(srv_name::SERVER_GET_EMR_ITEM, &ContextManager::getEmrItemCb, this);
  get_emr_vector_server_ = nh_.advertiseService(srv_name::SERVER_GET_EMR_VECTOR, &ContextManager::getEmrVectorCb, this);
  
  // Request remote EMR configurations
  emr_syncer_.requestRemoteConfigs();
  emr_sync_timer = nh_.createTimer(ros::Duration(1), &ContextManager::timerCallback, this);
  
  TEMOTO_INFO_("Context Manager is ready.");
}

// TODO: Do we need this? 
void ContextManager::timerCallback(const ros::TimerEvent&)
{
  // Request remote EMR configurations
  TEMOTO_DEBUG_STREAM_("Syncing EMR");
  emr_syncer_.requestRemoteConfigs();
}
/*
 * EMR synchronization callback
 */
void ContextManager::emrSyncCb(const temoto_core::ConfigSync& msg, const Items& payload)
{
  if (msg.action == temoto_core::trr::sync_action::REQUEST_CONFIG)
  {
    advertiseEmr();
    return;
  }

  // Add or update objects
  if (msg.action == temoto_core::trr::sync_action::ADVERTISE_CONFIG)
  {
    TEMOTO_DEBUG_("Received a payload.");
    updateEmr(payload, true);
  }
}

Items ContextManager::updateEmr(const Items& items_to_add, bool from_other_manager, bool update_time)
{
  
  // Keep track of failed add/update attempts
  std::vector<ItemContainer> failed_items = emr_interface->updateEmr(items_to_add, update_time);

  // If this object was added by its own namespace, then advertise this config to other managers
  if (!from_other_manager)
  {
    TEMOTO_INFO_("Advertising EMR to other namespaces.");
    advertiseEmr(); 
  }
  return failed_items;
}

/*
 * Advertise all objects
 */

void ContextManager::advertiseEmr()
{
  // Publish all items 
  Items items_payload = emr_interface->EmrToVector();
  // If there is something to send, advertise.
  if (items_payload.size()) 
  {
    emr_syncer_.advertise(items_payload);
  }
}

template <class Container>
std::string ContextManager::parseContainerType()
{
  if (std::is_same<Container, temoto_context_manager::ObjectContainer>::value) 
  {
    return emr_ros_interface::emr_containers::OBJECT;
  }
  else if (std::is_same<Container, temoto_context_manager::MapContainer>::value) 
  {
    return emr_ros_interface::emr_containers::MAP;
  }
  else if (std::is_same<Container, temoto_context_manager::ComponentContainer>::value) 
  {
    return emr_ros_interface::emr_containers::COMPONENT;
  }
  else if (std::is_same<Container, temoto_context_manager::RobotContainer>::value) 
  {
    return emr_ros_interface::emr_containers::ROBOT;
  }
  TEMOTO_ERROR_STREAM_("UNRECOGNIZED TYPE");
  return "FAULTY_TYPE";
}

bool ContextManager::getEmrItem(const std::string& name, std::string type, ItemContainer& container)
{
  std::string real_type = emr_interface->getTypeByName(name);
  // Check if requested type matches real type
  if (real_type == type) 
  {
    if (type == emr_ros_interface::emr_containers::OBJECT) 
    {
      auto rospl = emr_interface->getObject(name);
      container.serialized_container = temoto_core::serializeROSmsg(rospl);
      container.type = type;
      return true;
    }
    else if (type == emr_ros_interface::emr_containers::MAP) 
    {
      auto rospl = emr_interface->getMap(name);
      container.serialized_container = temoto_core::serializeROSmsg(rospl);
      container.type = type;
      return true;
    }
    else if (type == emr_ros_interface::emr_containers::COMPONENT) 
    {
      auto rospl = emr_interface->getComponent(name);
      container.serialized_container = temoto_core::serializeROSmsg(rospl);
      container.type = type;
      return true;
    }
    else if (type == emr_ros_interface::emr_containers::ROBOT) 
    {
      auto rospl = emr_interface->getRobot(name);
      container.serialized_container = temoto_core::serializeROSmsg(rospl);
      container.type = type;
      return true;
    }
    else
    {
      TEMOTO_ERROR_STREAM_("Unrecognized container type specified: " << type << std::endl);
      return false;
    }
  }
  else
  {
    TEMOTO_ERROR_STREAM_("Wrong type requested for EMR node with name: " << name << std::endl);
    TEMOTO_ERROR_STREAM_("Requested type: " << type << std::endl);
    TEMOTO_ERROR_STREAM_("Actual type: "<< real_type << std::endl);
    return false;
  }
}

bool ContextManager::getEmrVectorCb(GetEMRVector::Request& req, GetEMRVector::Response& res)
{
  res.items = emr_interface->EmrToVector();
  return true;
}

/*
 * Callback for adding objects
 */
bool ContextManager::updateEmrCb(UpdateEmr::Request& req, UpdateEmr::Response& res)
{
  (void)res; // Suppress "unused variable" compiler warnings
  TEMOTO_INFO_("Received a request to add %ld item(s) to the EMR.", req.items.size());

  res.failed_items = ContextManager::updateEmr(req.items, false);
  return true;
}

bool ContextManager::getEmrItemCb(GetEMRItem::Request& req, GetEMRItem::Response& res)
{
  TEMOTO_INFO_STREAM_("Received a request to get item: " << req.name << "from the EMR." << std::endl);
  ItemContainer nc;
  
  res.success = ContextManager::getEmrItem(req.name, req.type, nc);
  res.item = nc;
  TEMOTO_WARN_STREAM_("t1 " << res.success);
  return true;
}

}  // namespace temoto_context_manager
