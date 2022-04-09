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

#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_H

#include "temoto_context_manager/context_manager_services.h"
#include "temoto_context_manager/context_manager_containers.h"
#include "temoto_context_manager/env_model_interface.h"
#include "temoto_context_manager/emr_ros_interface.h"
#include "temoto_core/trr/config_synchronizer.h"
#include "temoto_core/common/base_subsystem.h" // TODO: deprecated stuff, get rid of it asap
#include "rr/ros1_resource_registrar.h"

namespace temoto_context_manager
{

class ContextManager : public temoto_core::BaseSubsystem
{
public:
  ContextManager();

private:

  void emrSyncCb(const temoto_core::ConfigSync& msg, const Items& payload);
  
  bool updateEmrCb(UpdateEmr::Request& req, UpdateEmr::Response& res);

  bool getEmrItemCb(GetEMRItem::Request& req, GetEMRItem::Response& res);

  bool getEmrVectorCb(GetEMRVector::Request& req, GetEMRVector::Response& res);

  /**
   * @brief Get node as nodecontainer from EMR
   * 
   * @param name 
   * @param container 
   * @return true 
   * @return false 
   */
  template <class Container> 
  bool getEmrItemHelper(const std::string& name, std::string type, ItemContainer& container);

  /**
   * @brief Get an item from EMR 
   * 
   * @param name 
   * @param type 
   * @param container 
   * @return true 
   * @return false 
   */
  bool getEmrItem(const std::string& name, std::string type, ItemContainer& container);

  /**
   * @brief Update the EMR structure with new information
   * 
   * @param items_to_add 
   * @param from_other_manager 
   * @return Items that could not be added
   */
  Items updateEmr(const Items & items_to_add, bool from_other_manager, bool update_time=false);
  
  /**
   * @brief Advertise the EMR state through the config syncer
   * 
   */
  void advertiseEmr();

  template <class Container>
  std::string parseContainerType();

  void timerCallback(const ros::TimerEvent&);

  ros::NodeHandle nh_;

  ros::ServiceServer update_emr_server_;

  ros::ServiceServer get_emr_item_server_;

  ros::ServiceServer get_emr_vector_server_;

  ObjectPtrs objects_;

  emr::EnvironmentModelRepository env_model_repository_;

  std::shared_ptr<EnvModelInterface> emr_interface; 

  ros::Timer emr_sync_timer;

  // TODO: remove. its currently here so that rr_core lib would get linked to
  // the context manager exec. otherwise the log macros dont work
  temoto_resource_registrar::ResourceRegistrarRos1 resource_registrar_;

  // Configuration syncer that manages external resource descriptions and synchronizes them
  // between all other (context) managers
  temoto_core::trr::ConfigSynchronizer<ContextManager, Items> emr_syncer_;

  // std::map<std::string, std::string> parameter_map_ =
  // {
  //   {"frame_id", emr_ros_interface::emr_containers::COMPONENT},
  //   {"odom_frame_id", emr_ros_interface::emr_containers::ROBOT},
  //   {"base_frame_id", emr_ros_interface::emr_containers::ROBOT}
  // };
};

} // temoto_context_manager namespace

#endif
