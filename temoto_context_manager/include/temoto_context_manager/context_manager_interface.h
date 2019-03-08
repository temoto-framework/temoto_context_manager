#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_INTERFACE_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_INTERFACE_H

#include "temoto_core/rmp/resource_manager.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/console_colors.h"
#include "temoto_core/common/topic_container.h"
#include "temoto_nlp/base_task/base_task.h"
#include "temoto_context_manager/context_manager_services.h"
#include "temoto_context_manager/MapContainer.h"
#include "temoto_core/common/ros_serialization.h"

#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "human_msgs/Hands.h"

#include <vector>

namespace temoto_context_manager
{

template <class OwnerTask>
class ContextManagerInterface : public temoto_core::BaseSubsystem
{
public:

  ContextManagerInterface()
  {
    class_name_ = __func__;
  }

  void initialize(temoto_nlp::BaseTask* task)
  {
    initializeBase(task);
    log_group_ = "interfaces." + task->getPackageName();
    name_ = task->getName() + "/context_manager_interface";

    // create resource manager
    resource_manager_ = std::unique_ptr<temoto_core::rmp::ResourceManager<ContextManagerInterface>>(new temoto_core::rmp::ResourceManager<ContextManagerInterface>(name_, this));

    // ensure that resource_manager was created
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // register status callback function
    resource_manager_->registerStatusCb(&ContextManagerInterface::statusInfoCb);

    // Add EMR service client
    update_EMR_client_ = nh_.serviceClient<UpdateEMR>(srv_name::SERVER_UPDATE_EMR);
    get_EMR_node_client_ = nh_.serviceClient<GetEMRNode>(srv_name::SERVER_GET_EMR_NODE);
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
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL , "The container is missing a name");
    }
    else
    {
      GetEMRNode srv_msg;
      srv_msg.request.name = name;
      if (std::is_same<Container, ObjectContainer>::value) 
      {
        srv_msg.request.type = "OBJECT";
      }
      else if (std::is_same<Container, MapContainer>::value) 
      {
        srv_msg.request.type = "MAP";
      }
      if (!get_EMR_node_client_.call<GetEMRNode>(srv_msg)) {
        throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to call the server");
      }
      TEMOTO_INFO("Got a response! ");
      if (srv_msg.response.success) 
      {
        container = temoto_core::deserializeROSmsg<Container>(
                                  srv_msg.response.node.serialized_container);
        TEMOTO_INFO("Got a response! ");
      }
      else
      {
        TEMOTO_ERROR_STREAM("Invalid EMR type requested for node.");
      }
    }
    return container;
    
  }
  int getNumber(const int number)
  {
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Create a GetNumber service message
    GetNumber srv_msg;
    srv_msg.request.requested_int = number;

    // Call the server
    try
    {
      resource_manager_->template call<GetNumber>( srv_name::MANAGER
                                                           , srv_name::GET_NUMBER_SERVER
                                                           , srv_msg);
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    return srv_msg.response.responded_int;
  }

  /**
   * @brief startTracker
   * @param tracker_category
   * @return
   */
  temoto_core::TopicContainer startTracker(std::string tracker_category)
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Start filling out the LoadTracker message
    LoadTracker load_tracker_msg;
    load_tracker_msg.request.tracker_category = tracker_category;

    try
    {
      resource_manager_->template call<LoadTracker>(srv_name::MANAGER_2,
                                                              srv_name::TRACKER_SERVER,
                                                              load_tracker_msg);
      allocated_trackers_.push_back(load_tracker_msg);
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    temoto_core::TopicContainer topics_to_return;
    topics_to_return.setOutputTopicsByKeyValue(load_tracker_msg.response.output_topics);

    return topics_to_return;
  }

  std::string trackObject(std::string object_name)
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    // Start filling out the TrackObject message
    TrackObject track_object_msg;
    track_object_msg.request.object_name = object_name;

    try
    {
      resource_manager_->template call<TrackObject>(srv_name::MANAGER,
                                                              srv_name::TRACK_OBJECT_SERVER,
                                                              track_object_msg);

      allocated_track_objects_.push_back(track_object_msg);
      return track_object_msg.response.object_topic;
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }
  /**
   * @brief Add single container to EMR
   * 
   * @tparam Container 
   * @param container 
   */
  template <class Container>
  void addToEMR(const Container& container)
  {
    std::vector<Container> containers;
    containers.push_back(container);
    addToEMR(containers);
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
  void addToEMR(const std::vector<Container> & containers)
  {
    std::vector<temoto_context_manager::NodeContainer> node_containers;
    for (const auto& container : containers)
    {
      if (container.name == "")
      {
        throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL , "The container is missing a name");
      }
      else
      {
        temoto_context_manager::NodeContainer nc;
        // Check the type of the container
        if (std::is_same<Container, ObjectContainer>::value) 
        {
          nc.type = "OBJECT";
        }
        else if (std::is_same<Container, MapContainer>::value) 
        {
          nc.type = "MAP";
        }
        nc.serialized_container = temoto_core::serializeROSmsg(container);
        node_containers.push_back(nc);
      }
      
    }
    UpdateEMR update_EMR_srvmsg;
    update_EMR_srvmsg.request.nodes = node_containers;

    if (!update_EMR_client_.call<UpdateEMR>(update_EMR_srvmsg)) 
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Failed to call the server");
    }
    for (auto node_container : update_EMR_srvmsg.response.failed_nodes)
    {
      if (node_container.type == "OBJECT") {
        auto container = temoto_core::deserializeROSmsg<ObjectContainer>
                                (node_container.serialized_container);
        TEMOTO_INFO_STREAM("Failed to add node: " << container.name << std::endl);
      }
      else if (node_container.type == "MAP")
      {
        auto container = temoto_core::deserializeROSmsg<ObjectContainer>
                                (node_container.serialized_container);
        TEMOTO_INFO_STREAM("Failed to add node: " << container.name << std::endl);
      }
    }
  }

  /**
   * @brief stopAllocatedServices
   * @return
   */
  bool stopAllocatedServices()
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    try
    {
      // remove all connections, which were created via call() function
      resource_manager_->unloadClients();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
  }

  void statusInfoCb(temoto_core::ResourceStatus& srv)
  {
    // Validate the interface
    try
    {
      validateInterface();
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }

    TEMOTO_DEBUG("Received status information");
    TEMOTO_DEBUG_STREAM(srv.request);
    // if any resource should fail, just unload it and try again
    // there is a chance that sensor manager gives us better sensor this time
    if (srv.request.status_code == temoto_core::rmp::status_codes::FAILED)
    {
      TEMOTO_WARN("Received a notification about a resource failure. Unloading and trying again");

      auto tracker_it = std::find_if(allocated_trackers_.begin(), allocated_trackers_.end(),
                                  [&](const LoadTracker& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });

      auto track_object_it = std::find_if(allocated_track_objects_.begin(), allocated_track_objects_.end(),
                                  [&](const TrackObject& sens) -> bool {
                                    return sens.response.rmp.resource_id == srv.request.resource_id;
                                  });

      // If the tracker was found then ...
      if (tracker_it != allocated_trackers_.end())
      {
        try
        {
          // ... unload it and ...
          TEMOTO_DEBUG("Unloading the tracker");
          resource_manager_->unloadClientResource(tracker_it->response.rmp.resource_id);

          // ... copy the output topics from the response into the output topics
          // of the request (since the user still wants to receive the data on the same topics) ...
          tracker_it->request.output_topics = tracker_it->response.output_topics;
          tracker_it->request.pipe_id = tracker_it->response.pipe_id;

          // ... and load an alternative tracker. This call automatically
          // updates the response in allocated trackers vector

          TEMOTO_DEBUG_STREAM("Trying to load an alternative tracker");

          resource_manager_->template call<LoadTracker>(srv_name::MANAGER_2,
                                                                  srv_name::TRACKER_SERVER,
                                                                  *tracker_it);
        }
        catch(temoto_core::error::ErrorStack& error_stack)
        {
          throw FORWARD_ERROR(error_stack);
        }
      }

      // If the tracker was found then ...
      else if (track_object_it != allocated_track_objects_.end())
      {
        try
        {
          // ... unload it and ...
          TEMOTO_DEBUG("Unloading the track object");
          resource_manager_->unloadClientResource(track_object_it->response.rmp.resource_id);

          TEMOTO_DEBUG_STREAM("Trying to resume tracking the " << track_object_it->request.object_name);

          resource_manager_->template call<TrackObject>(srv_name::MANAGER,
                                                                  srv_name::TRACK_OBJECT_SERVER,
                                                                  *track_object_it);
        }
        catch(temoto_core::error::ErrorStack& error_stack)
        {
          throw FORWARD_ERROR(error_stack);
        }
      }

      if (tracker_it != allocated_trackers_.end() &&
          track_object_it != allocated_track_objects_.end())
      {
        throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND, "Got resource_id that is not registered in this interface.");
      }
    }
  }


  ~ContextManagerInterface(){}

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  std::unique_ptr<temoto_core::rmp::ResourceManager<ContextManagerInterface>> resource_manager_;

  std::string name_; 
  temoto_nlp::BaseTask* task_;

  ros::NodeHandle nh_;
  ros::ServiceClient update_EMR_client_;
  ros::ServiceClient get_EMR_node_client_;

  std::vector<LoadTracker> allocated_trackers_;
  std::vector<TrackObject> allocated_track_objects_;

  /**
   * @brief validateInterface()
   * @param component_type
   */
  void validateInterface()
  {
    if(!resource_manager_)
    {
      throw CREATE_ERROR(temoto_core::error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }
};

} // namespace
#endif
