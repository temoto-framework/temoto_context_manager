#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_H

#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/reliability.h"
#include "temoto_core/rmp/resource_manager.h"
#include "temoto_core/rmp/config_synchronizer.h"

#include "temoto_context_manager/context_manager_services.h"
#include "temoto_context_manager/context_manager_containers.h"
#include "temoto_context_manager/env_model_repository.h"
#include "temoto_context_manager/emr_ros_interface.h"
#include "temoto_context_manager/emr_item_to_component_link.h"

#include "temoto_nlp/task_manager.h"
#include "temoto_component_manager/component_manager_services.h"

namespace temoto_context_manager
{

class ContextManager : public temoto_core::BaseSubsystem
{
public:
  ContextManager();

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  void loadTrackObjectCb(TrackObject::Request& req, TrackObject::Response& res);

  void unloadTrackObjectCb(TrackObject::Request& req, TrackObject::Response& res);

  void emrSyncCb(const temoto_core::ConfigSync& msg, const Items& payload);
  
  bool updateEmrCb(UpdateEmr::Request& req, UpdateEmr::Response& res);

  void trackedObjectsSyncCb(const temoto_core::ConfigSync& msg, const std::string& payload);

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

  ObjectPtr findObject(std::string object_name);

  void statusCb1(temoto_core::ResourceStatus& srv);

  void statusCb2(temoto_core::ResourceStatus& srv);

  void addDetectionMethod(std::string detection_method);

  void addDetectionMethods(std::vector<std::string> detection_methods);

  std::vector<std::string> getOrderedDetectionMethods();

  std::vector<std::string> getItemDetectionMethods(const std::string& name);

  void startComponentToEMRLinker();

  // Resource manager for handling servers and clients
  temoto_core::rmp::ResourceManager<ContextManager> resource_manager_1_;

  /*
   * Resource manager for handling servers and clients.
   * TODO: The second manager is used for making RMP calls within the same manager. If the same
   * resouce manager is used for calling servers managed by the same manager, the calls will lock
   */
  temoto_core::rmp::ResourceManager<ContextManager> resource_manager_2_;

  ros::NodeHandle nh_;

  ros::ServiceServer update_emr_server_;

  ros::ServiceServer get_emr_item_server_;

  ObjectPtrs objects_;

  std::map<int, std::string> m_tracked_objects_local_;

  std::map<std::string, std::string> m_tracked_objects_remote_;

  emr::EnvironmentModelRepository env_model_repository_;

  emr_ros_interface::EmrRosInterface emr_interface{env_model_repository_, temoto_core::common::getTemotoNamespace()};

  // Configuration syncer that manages external resource descriptions and synchronizes them
  // between all other (context) managers
  temoto_core::rmp::ConfigSynchronizer<ContextManager, Items> emr_syncer_;

  temoto_core::rmp::ConfigSynchronizer<ContextManager, std::string> tracked_objects_syncer_;

  temoto_nlp::TaskManager action_engine_;

  std::map<std::string, temoto_core::Reliability> detection_method_history_;

  std::pair<int, std::string> active_detection_method_;
};

} // temoto_context_manager namespace

#endif
