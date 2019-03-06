#include "ros/package.h"
#include "temoto_context_manager/context_manager.h"
#include <algorithm>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "temoto_core/common/ros_serialization.h"

namespace temoto_context_manager
{

ContextManager::ContextManager()
  : temoto_core::BaseSubsystem("temoto_context_manager", temoto_core::error::Subsystem::CONTEXT_MANAGER, __func__)
  , resource_manager_1_(srv_name::MANAGER, this)
  , resource_manager_2_(srv_name::MANAGER_2, this)
  , tracked_objects_syncer_(srv_name::MANAGER, srv_name::SYNC_TRACKED_OBJECTS_TOPIC, &ContextManager::trackedObjectsSyncCb, this)
  , EMR_syncer_(srv_name::MANAGER, srv_name::SYNC_OBJECTS_TOPIC, &ContextManager::EMRSyncCb, this)
  , tracker_core_(this, false, ros::package::getPath(ROS_PACKAGE_NAME) + "/config/action_dst.yaml")
{
  /*
   * Start the servers
   */

  // TODO: This is just an example service
  resource_manager_1_.addServer<GetNumber>( srv_name::GET_NUMBER_SERVER
                                                    , &ContextManager::loadGetNumberCb
                                                    , &ContextManager::unloadGetNumberCb);

  // Object tracking service
  resource_manager_1_.addServer<TrackObject>(srv_name::TRACK_OBJECT_SERVER
                                                    , &ContextManager::loadTrackObjectCb
                                                    , &ContextManager::unloadTrackObjectCb);

  // Tracker setup service
  resource_manager_2_.addServer<LoadTracker>(srv_name::TRACKER_SERVER
                                                    , &ContextManager::loadTrackerCb
                                                    , &ContextManager::unloadTrackerCb);

  // Register callback for status info
  resource_manager_1_.registerStatusCb(&ContextManager::statusCb1);
  resource_manager_2_.registerStatusCb(&ContextManager::statusCb2);

  // "Update EMR" server
  update_emr_server_ = nh_.advertiseService(srv_name::SERVER_UPDATE_EMR, &ContextManager::updateEMRCb, this);
  
  // Request remote EMR configurations
  EMR_syncer_.requestRemoteConfigs();

  /*
   * Process the tracking methods that are described in an external YAML file
   */

  // Path to the trackers YAML file
  std::string yaml_filename = ros::package::getPath(ROS_PACKAGE_NAME) + "/conf/" +
                                "tracking_methods.yaml";

  // Parse the trackes
  parseTrackers(yaml_filename);

  // Print out the trackers
//  for (auto& tracker_category : categorized_trackers_)
//  {
//    std::cout << "CATEGORY: " << tracker_category.first << std::endl;
//    for (auto& tracking_method : tracker_category.second)
//    {
//      std::cout << tracking_method.toString() << std::endl;
//    }
//  }
  
  TEMOTO_INFO("Context Manager is ready.");
}

/*
 * Implementation of the GetNumber service
 */ 
void ContextManager::loadGetNumberCb( GetNumber::Request& req
                                    , GetNumber::Response& res)
{
  TEMOTO_INFO_STREAM("Received a request to load number '" << req.requested_int << "'");
  res.responded_int = req.requested_int;
}

/*
 * Implementation of the unload GetNumber service
 */ 
void ContextManager::unloadGetNumberCb( GetNumber::Request& req
                                      , GetNumber::Response& res)
{
  (void)res; // Suppress "unused parameter" compiler warnings
  TEMOTO_INFO_STREAM("Received a request to UNload number '" << req.requested_int << "'");
}

/*
 * EMR synchronization callback
 */
void ContextManager::EMRSyncCb(const temoto_core::ConfigSync& msg, const Nodes& payload)
{
  if (msg.action == temoto_core::rmp::sync_action::REQUEST_CONFIG)
  {
    advertiseEMR();
    return;
  }

  // Add or update objects
  if (msg.action == temoto_core::rmp::sync_action::ADVERTISE_CONFIG)
  {
    TEMOTO_DEBUG("Received a payload.");
    updateEMR(payload, true);
  }
}
/**
 * @brief Function to traverse and print out every node name in the tree
 * 
 * @param root 
 */
void ContextManager::traverseEMR(const emr::Node& root)
{
  TEMOTO_DEBUG_STREAM(root.getPayload()->getName());
  std::shared_ptr<emr::ROSPayload<ObjectContainer>> msgptr = std::dynamic_pointer_cast<emr::ROSPayload<ObjectContainer>>(root.getPayload());
  TEMOTO_DEBUG_STREAM("tag_id: " << msgptr->getPayload().tag_id);
  std::vector<std::shared_ptr<emr::Node>> children = root.getChildren();
  for (uint32_t i = 0; i < children.size(); i++)
  {
    traverseEMR(*children[i]);
  }
}

/*
 * Tracked objects synchronization callback
 */
void ContextManager::trackedObjectsSyncCb(const temoto_core::ConfigSync& msg, const std::string& payload)
{
  if (msg.action == temoto_core::rmp::sync_action::ADVERTISE_CONFIG)
  {
    TEMOTO_DEBUG_STREAM("Received a message, that '" << payload << "' is tracked by '"
                        << msg.temoto_namespace << "'.");

    // Add a notion about a object that is being tracked
    m_tracked_objects_remote_[payload] = msg.temoto_namespace;
  }
  else
  if (msg.action == temoto_core::rmp::sync_action::REMOVE_CONFIG)
  {
    TEMOTO_DEBUG_STREAM("Received a message, that '" << payload << "' is not tracked by '"
                        << msg.temoto_namespace << "' anymore.");

    // Remove a notion about a object that is being tracked
    m_tracked_objects_remote_.erase(payload);
  }
}

Nodes ContextManager::EMRtoVector(const emr::EnvironmentModelRepository& emr)
{
  Nodes nodes;
  std::vector<std::shared_ptr<emr::Node>> root_nodes = emr.getRootNodes();
  for (const auto& node : root_nodes)
  {
    EMRtoVectorHelper(*node, nodes);
  }
  return nodes;
}

void ContextManager::EMRtoVectorHelper(const emr::Node& currentNode, Nodes& nodes)
{
  // Create empty container and fill it based on the payload
  NodeContainer nc;
  
  nc.type = currentNode.getPayload()->getType();

  // Get the node payload as ROS msg
  // To do this we dynamically cast the base class to the appropriate
  //   derived class and call the getPayload() method
  if (nc.type == "OBJECT") 
  {
    ObjectContainer rospl = 
      std::dynamic_pointer_cast<emr::ROSPayload<ObjectContainer>>(currentNode.getPayload())
        ->getPayload();
    nc.serialized_container = temoto_core::serializeROSmsg(rospl);
    nodes.push_back(nc);
  }
  else if (nc.type == "MAP") 
  {
    MapContainer rospl = 
      std::dynamic_pointer_cast<emr::ROSPayload<MapContainer>>(currentNode.getPayload())
        ->getPayload();
    nc.serialized_container = temoto_core::serializeROSmsg(rospl);
    nodes.push_back(nc);
  }
  else
  {
    TEMOTO_ERROR_STREAM("Wrong type of container @ EMRtoVectorHelper: " << nc.type);
    return;
  }

  std::vector<std::shared_ptr<emr::Node>> children = currentNode.getChildren();
  for (uint32_t i = 0; i < children.size(); i++)
  {
    EMRtoVectorHelper(*children[i], nodes);
  }
}

template <class Container>
void ContextManager::addOrUpdateEMRNode(const Container & container, const std::string& container_type)
{
  emr::ROSPayload<Container> rospl = emr::ROSPayload<Container>(container);
  rospl.setType(container_type);
  std::string name = container.name;
  std::string parent = container.parent;

  // Check for empty name field
  // Move these to the context manager interface maybe? TBD
  if (name == "") 
  {
    TEMOTO_ERROR_STREAM("Empty string not allowed as EMR node name!");
    return;
  }
  // Check if the parent exists
  if ((!parent.empty()) && (!env_model_repository_.hasNode(parent))) 
  {
    TEMOTO_ERROR_STREAM("No parent with name " << parent << " found in EMR!");
    return;
  }
  
  // Check if the object has to be added or updated
  if (!env_model_repository_.hasNode(name)) 
  {
    // Add the new node
    std::shared_ptr<emr::ROSPayload<Container>> plptr = std::make_shared<emr::ROSPayload<Container>>(rospl);
    env_model_repository_.addNode(name, parent, plptr);
  }
  else
  {
    // Update the node information
    std::shared_ptr<emr::ROSPayload<Container>> plptr = std::make_shared<emr::ROSPayload<Container>>(rospl);
    env_model_repository_.updateNode(name, plptr);
  }
  traverseEMR(*env_model_repository_.getNodeByName("Object1"));
}

void ContextManager::updateEMR(const Nodes& nodes_to_add, bool from_other_manager)
{

  for (auto node_container : nodes_to_add)
  {
    if (node_container.type == "OBJECT") 
    {
      // Deserialize into an ObjectContainer object and add to EMR
      ObjectContainer oc = 
        temoto_core::deserializeROSmsg<ObjectContainer>(node_container.serialized_container);
      addOrUpdateEMRNode(oc, "OBJECT");
    }
    else if (node_container.type == "MAP") 
    {
      // Deserialize into an MapContainer object and add to EMR
      MapContainer mc = 
        temoto_core::deserializeROSmsg<MapContainer>(node_container.serialized_container);
      addOrUpdateEMRNode(mc, "MAP");
    }
    else
    {
      TEMOTO_ERROR_STREAM("Wrong type " << node_container.type.c_str() << "specified for EMR node");
      return;
    }
  }

  // If this object was added by its own namespace, then advertise this config to other managers
  if (!from_other_manager)
  {
    TEMOTO_INFO("Advertising EMR to other namespaces.");
    advertiseEMR(); 
  }
}

/*
 * Advertise all objects
 */
// 
void ContextManager::advertiseEMR()
{
  // Publish all nodes 
  Nodes nodes_payload = EMRtoVector(env_model_repository_);

  // If there is something to send, advertise.
  if (nodes_payload.size()) 
  {
    EMR_syncer_.advertise(nodes_payload);
  }
  
}

/*
 * Find object
 */
ObjectPtr ContextManager::findObject(std::string object_name)
{
  for (auto& object : objects_)
  {
    if (object->name == object_name)
    {
      return object;
    }
  }

  // Throw an error if no objects were found
  throw CREATE_ERROR(temoto_core::error::Code::UNKNOWN_OBJECT, "The requested object is unknown");
}


/*
 * Callback for adding objects
 */
bool ContextManager::updateEMRCb(UpdateEMR::Request& req, UpdateEMR::Response& res)
{
  (void)res; // Suppress "unused variable" compiler warnings
  TEMOTO_INFO("Received a request to add %ld node(s) to the EMR.", req.nodes.size());

  ContextManager::updateEMR(req.nodes, false);

  return true;
}

/*
 * Server for tracking objects
 */
void ContextManager::loadTrackObjectCb(TrackObject::Request& req, TrackObject::Response& res)
{
  try
  {
    // Replace all spaces in the name with the underscore character
    std::string object_name_no_space = req.object_name;
    std::replace(object_name_no_space.begin(), object_name_no_space.end(), ' ', '_');

    TEMOTO_INFO_STREAM("Received a request to track an object named: '" << object_name_no_space << "'");

    /*
     * Check if this object is already tracked in other temoto namespace
     */
    std::string remote_temoto_namespace = m_tracked_objects_remote_[object_name_no_space];

    if (!remote_temoto_namespace.empty())
    {
      TEMOTO_DEBUG_STREAM("The object '" << object_name_no_space << "' is alerady tracked by '"
                          << remote_temoto_namespace << "'. Forwarding the request.");

      TrackObject track_object_msg;
      track_object_msg.request = req;

      // Send the request to the remote namespace
      resource_manager_2_.template call<TrackObject>(srv_name::MANAGER,
                                                     srv_name::TRACK_OBJECT_SERVER,
                                                     track_object_msg,
                                                     temoto_core::rmp::FailureBehavior::NONE,
                                                     remote_temoto_namespace);

      res = track_object_msg.response;
      return;
    }

    // Look if the requested object is described in the object database
    ObjectPtr requested_object = findObject(object_name_no_space);

    TEMOTO_INFO_STREAM("The requested object is known");

    /*
     * Start a tracker that can be used to detect the requested object
     */
    LoadTracker load_tracker_msg;
    addDetectionMethods(requested_object->detection_methods);
    std::vector<std::string> detection_methods = getOrderedDetectionMethods();

    for (auto dm : detection_methods)
    {
      std::cout << dm << " !!!!!!!!!!!!!!!! " << std::endl;
    }

    std::string selected_tracker;

    // Loop over different tracker categories and try to load one. The loop is iterated either until
    // a tracker is succesfully loaded or options are exhausted (failure)
    for (auto& tracker_category : detection_methods)
    {
      try
      {
        TEMOTO_INFO_STREAM("Trying to track the " << object_name_no_space
                           << " via '"<< tracker_category << "'");

        load_tracker_msg = LoadTracker();
        load_tracker_msg.request.tracker_category = tracker_category;
        resource_manager_1_.call<LoadTracker>(srv_name::MANAGER_2,
                                                        srv_name::TRACKER_SERVER,
                                                        load_tracker_msg);

        selected_tracker = tracker_category;
        detection_method_history_[tracker_category].adjustReliability();
        active_detection_method_.first = load_tracker_msg.response.rmp.resource_id;
        active_detection_method_.second = tracker_category;
        break;
      }
      catch (temoto_core::error::ErrorStack& error_stack)
      {
        detection_method_history_[tracker_category].adjustReliability(0);

        // If a requested tracker was not found but there are other options
        // available, then continue. Otherwise forward the error
        if (error_stack.front().code == static_cast<int>(temoto_core::error::Code::NO_TRACKERS_FOUND) &&
            &tracker_category != &detection_methods.back())
        {
          continue;
        }

        throw FORWARD_ERROR(error_stack);
      }
    }

    /*
     * Start the specific object tracker. Since there are different general object
     * tracking methods and each tracker outputs different types of data, then
     * the specific tracking has to be set up based on the general tracker. For example
     * a general tracker, e.g. AR tag detector, publshes data about detected tags. The
     * specific tracker has to subscribe to the detected tags topic and since the
     * tags are differentiated by the tag ID, the specific tracker has to know the ID
     * beforehand.
     */

    /*
     * Get the topic where the tracker publishes its output data
     */
    temoto_core::TopicContainer tracker_topics;
    tracker_topics.setOutputTopicsByKeyValue(load_tracker_msg.response.output_topics);

    // Topic where the information about the required object is going to be published.
    std::string tracked_object_topic = temoto_core::common::getAbsolutePath("object_tracker/" + object_name_no_space);

    /*
     * AR tag based object tracker setup
     */
    if (selected_tracker == "artags")
    {
      TEMOTO_DEBUG_STREAM("Using AR-tag based tracking");

      // Get the AR tag data dopic
      std::string tracker_data_topic = tracker_topics.getOutputTopic("marker_data");

      /*
       * TTP related stuff up ahead: A semantic frame is manually created. Based on that SF
       * a SF tree is created, given that an action implementation, that corresponds to the
       * manually created SF, exists. The specific tracker task is started and it continues
       * running in the background until its ordered to stop.
       */

      std::string action = "track";
      temoto_nlp::Subjects subjects;

      // Subject that will contain the name of the tracked object.
      // Necessary when the tracker has to be stopped
      temoto_nlp::Subject sub_0("what", object_name_no_space);

      // Subject that will contain the data necessary for the specific tracker
      temoto_nlp::Subject sub_1("what", "artag data");

      // Topic from where the raw AR tag tracker data comes from
      sub_1.addData("topic", tracker_data_topic);

      // Topic where the AImp must publish the data about the tracked object
      sub_1.addData("topic", tracked_object_topic);

      // This object will be updated inside the tracking AImp (action implementation)
      sub_1.addData("pointer", boost::any_cast<ObjectPtr>(requested_object));

      subjects.push_back(sub_0);
      subjects.push_back(sub_1);

      // Create a SF
      std::vector<temoto_nlp::TaskDescriptor> task_descriptors;
      task_descriptors.emplace_back(action, subjects);
      task_descriptors[0].setActionStemmed(action);

      // Create a sematic frame tree
      temoto_nlp::TaskTree sft = temoto_nlp::SFTBuilder::build(task_descriptors);

      // Get the root node of the tree
      temoto_nlp::TaskTreeNode& root_node = sft.getRootNode();
      sft.printTaskDescriptors(root_node);

      // Execute the SFT
      tracker_core_.executeSFT(std::move(sft));

      // Put the object into the list of tracked objects. This is used later
      // for stopping the tracker
      m_tracked_objects_local_[res.rmp.resource_id] = object_name_no_space;
    }

    /*
     * Hand based object tracker setup
     */
    if (selected_tracker == "hands")
    {
      TEMOTO_DEBUG_STREAM("Using hand based tracking");

      // Get the AR tag data dopic
      std::string tracker_data_topic = tracker_topics.getOutputTopic("handtracker_data");

      /*
       * TTP related stuff up ahead: A semantic frame is manually created. Based on that SF
       * a SF tree is created, given that an action implementation, that corresponds to the
       * manually created SF, exists. The specific tracker task is started and it continues
       * running in the background until its ordered to stop.
       */

      std::string action = "track";
      temoto_nlp::Subjects subjects;

      // Subject that will contain the name of the tracked object.
      // Necessary when the tracker has to be stopped
      temoto_nlp::Subject sub_0("what", object_name_no_space);

      // Subject that will contain the data necessary for the specific tracker
      temoto_nlp::Subject sub_1("what", "hand data");

      // Topic from where the raw hand tracker data comes from
      sub_1.addData("topic", tracker_data_topic);

      // Topic where the AImp must publish the data about the tracked object
      sub_1.addData("topic", tracked_object_topic);

      // This object will be updated inside the tracking AImp (action implementation)
      sub_1.addData("pointer", boost::any_cast<ObjectPtr>(requested_object));

      subjects.push_back(sub_0);
      subjects.push_back(sub_1);

      // Create a SF
      std::vector<temoto_nlp::TaskDescriptor> task_descriptors;
      task_descriptors.emplace_back(action, subjects);
      task_descriptors[0].setActionStemmed(action);

      // Create a sematic frame tree
      temoto_nlp::TaskTree sft = temoto_nlp::SFTBuilder::build(task_descriptors);

      // Get the root node of the tree
      temoto_nlp::TaskTreeNode& root_node = sft.getRootNode();
      sft.printTaskDescriptors(root_node);

      // Execute the SFT
      tracker_core_.executeSFT(std::move(sft));

      // Put the object into the list of tracked objects. This is used later
      // for stopping the tracker
      m_tracked_objects_local_[res.rmp.resource_id] = object_name_no_space;
    }

    res.object_topic = tracked_object_topic;

    // Let context managers in other namespaces know, that this object is being tracked
    tracked_objects_syncer_.advertise(object_name_no_space);

  }
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

/*
 * Unload the track object
 */
void ContextManager::unloadTrackObjectCb(TrackObject::Request& req,
                                         TrackObject::Response& res)
{
  /*
   * Stopping tracking the object based on its name
   */
  try
  {
    // Check if the object is tracked locally or by a remote manager
    if (!m_tracked_objects_remote_[req.object_name].empty())
    {
      // The object is tracked by a remote manager
      return;
    }

    // Get the name of the tracked object
    std::string tracked_object = m_tracked_objects_local_[res.rmp.resource_id];

    if (tracked_object.empty())
    {
      throw CREATE_ERROR(temoto_core::error::Code::NO_TRACKERS_FOUND, std::string("The object '") +
                         req.object_name + "' is not tracked");
    }

    TEMOTO_DEBUG_STREAM("Received a request to stop tracking an object named: '"
                        << tracked_object << "'");

    // Stop tracking the object
    tracker_core_.stopTask("", tracked_object);

    // Erase the object from the map of tracked objects
    m_tracked_objects_local_.erase(res.rmp.resource_id);

    // Let context managers in other namespaces know, that this object is not tracked anymore
    tracked_objects_syncer_.advertise(tracked_object, temoto_core::rmp::sync_action::REMOVE_CONFIG);
  }
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

/**
 * @brief findTrackers
 * @param req
 * @return
 */
TrackerInfoPtrs ContextManager::findTrackers(LoadTracker::Request& req)
{
  // Get the tracking methods of the requested category
  auto tracker_category = categorized_trackers_.find(req.tracker_category);

  // Throw an error if the requested tracker category does not exist
  if (tracker_category == categorized_trackers_.end())
  {
    throw CREATE_ERROR(temoto_core::error::Code::NO_TRACKERS_FOUND, "No trackers found for the requested category");
  }

  // Get the trackers
  TrackerInfoPtrs trackers = tracker_category->second;

  // Check if there are any required types for the output topics of the tracker
  if (!req.output_topics.empty())
  {
    // Loop over trackers
    for (auto tracker_it = trackers.begin(); tracker_it != trackers.end(); /* empty */)
    {
      bool tracker_suitable = true;

      // Create a copy of the required topics
      std::vector<diagnostic_msgs::KeyValue> req_topic_types = req.output_topics;
      std::set<std::string> last_filter_topics = (*tracker_it)->getPipe().back().required_output_topic_types_;

      // The topics that the last filter of the pipe provides
      for (auto& last_filter_topic : last_filter_topics)
      {
        bool topic_found = false;

        // Compare the topic of the last filter with the required topics
        for (auto rtt_it = req_topic_types.begin(); rtt_it != req_topic_types.end(); rtt_it++)
        {
          if (rtt_it->key == last_filter_topic)
          {
            topic_found = true;
            req_topic_types.erase(rtt_it);
            break;
          }
        }

        // If the required topic was not found then break the loop and indicate
        // that the tracker is not suitable
        if (!topic_found)
        {
          tracker_suitable = false;
          break;
        }
      }

      // Remove the tracker if its last filter does not contain the required topic type
      if (!tracker_suitable)
      {
        trackers.erase(tracker_it);
      }
      else
      {
        tracker_it++;
      }
    }

    // If no tracker was suitable, then throw an error
    if (trackers.empty())
    {
      throw CREATE_ERROR(temoto_core::error::Code::NO_TRACKERS_FOUND, "No trackers found for the requested topic types");
    }
  }

  // Sort the trackers with decreasing reliability order
  std::sort( trackers.begin()
           , trackers.end()
           , [](const TrackerInfoPtr lhs, const TrackerInfoPtr rhs)
             {
               return lhs->reliability_.getReliability() >
                      rhs->reliability_.getReliability();
             });

  return trackers;
}

/*
 * Load tracker callback
 */
void ContextManager::loadTrackerCb(LoadTracker::Request& req,
                                   LoadTracker::Response& res)
{
  TEMOTO_INFO_STREAM("Received a request: \n" << req << std::endl);

  TrackerInfoPtrs trackers;
  try
  {
    // Get the trackers that follow the requested criteria
    trackers = findTrackers(req);
  }
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
  TEMOTO_DEBUG_STREAM("Found the requested tracker category.");

  /*
   * Loop over all possible tracking methods until somethin starts to work
   */
  for (TrackerInfoPtr tracker : trackers)
  {
    TEMOTO_DEBUG_STREAM("Trying tracker: \n" << tracker->toString().c_str());

    try
    {
      // Get the id of the pipe, if provided
      std::string pipe_id = req.pipe_id;

      // Create a new pipe id if it was not specified
      if (pipe_id == "")
      {
        // Create a unique pipe identifier string
        pipe_id = "pipe_" + std::to_string(pipe_id_generator_.generateID())
                + "_at_" + temoto_core::common::getTemotoNamespace();
      }

      /*
       * Build the pipe based on the number of filters. If the pipe
       * contains only one filter, then there are no constraints on
       * the ouptut topic types. But if the pipe contains multiple filters
       * then each preceding filter has to provide the topics that are
       * required by the proceding filter
       */
      temoto_core::TopicContainer required_topics;

      if (tracker->getPipeSize() > 1)
      {
        // TODO: If the right hand side of the "req_tops" is directly used in the proceeding
        // for-loop, then it crashes on the second loop. Not sure why.
        std::set<std::string> req_tops = tracker->getPipe().at(1).required_input_topic_types_;

        // Loop over requested topics
        for (auto& topic : req_tops)
        {
          required_topics.addOutputTopicType(topic);
        }
      }
      else
      {
        // TODO: If the right hand side of the "req_tops" is directly used in the proceeding
        // for-loop, then it crashes on the second loop. Not sure why.
        std::set<std::string> req_tops = tracker->getPipe().at(0).required_output_topic_types_;

        // Loop over requested topics
        for (auto& topic : req_tops)
        {
          required_topics.addOutputTopicType(topic);
        }
      }

      // Loop over the pipe
      std::vector<Filter> pipe = tracker->getPipe();

      // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
      std::vector<int> sub_resource_ids;

      for (unsigned int i=0; i<pipe.size(); i++)
      {
        /*
         * If the filter is a sensor
         */
        if (pipe.at(i).filter_category_ == "sensor")
        {
          // Compose the LoadSensor message
          temoto_component_manager::LoadComponent load_component_msg;
          load_component_msg.request.component_type = pipe.at(i).filter_type_;
          load_component_msg.request.output_topics = required_topics.outputTopicsAsKeyValues();

          // Call the Sensor Manager
          resource_manager_2_.call<temoto_component_manager::LoadComponent>(temoto_component_manager::srv_name::MANAGER,
                                                                            temoto_component_manager::srv_name::SERVER,
                                                                            load_component_msg);

          // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
          sub_resource_ids.push_back(load_component_msg.response.rmp.resource_id);

          required_topics.setInputTopicsByKeyValue(load_component_msg.response.output_topics);

          // This line is necessary if the pipe size is 1
          required_topics.setOutputTopicsByKeyValue(load_component_msg.response.output_topics);
        }

        /*
         * If the filter is an algorithm
         */
        else if (pipe.at(i).filter_category_ == "algorithm")
        {

          // Clear out the required output topics
          required_topics.clearOutputTopics();

          // If it is not the last filter then ...
          if (i != pipe.size()-1)
          {
            // ... get the requirements for the output topic types from the proceding filter
            for (auto& topic_type : pipe.at(i+1).required_input_topic_types_)
            {
              required_topics.addOutputTopic(topic_type, "/" + pipe_id + "/filter_" + std::to_string(i) + "/" + topic_type);
            }
          }
          else
          {
            // ... get the requirements for the output topics from own output topic requirements
            // TODO: throw if the "required_output_topic_types_" is empty
            for (auto& topic_type : pipe.at(i).required_output_topic_types_)
            {
              required_topics.addOutputTopic(topic_type, "/" + pipe_id + "/filter_" + std::to_string(i) + "/" + topic_type);
            }
          }

          // Compose the LoadAlgorithm message
          temoto_component_manager::LoadComponent load_component_msg;
          load_component_msg.request.component_type = pipe.at(i).filter_type_;
          load_component_msg.request.input_topics = required_topics.inputTopicsAsKeyValues();
          load_component_msg.request.output_topics = required_topics.outputTopicsAsKeyValues();

          // Call the Algorithm Manager
          resource_manager_2_.call<temoto_component_manager::LoadComponent>(temoto_component_manager::srv_name::MANAGER,
                                                                            temoto_component_manager::srv_name::SERVER,
                                                                            load_component_msg);

          // TODO: REMOVE AFTER RMP HAS THIS FUNCTIONALITY
          sub_resource_ids.push_back(load_component_msg.response.rmp.resource_id);

          required_topics.setInputTopicsByKeyValue(load_component_msg.response.output_topics);
        }
      }

      // Send the output topics of the last filter back via response
      res.output_topics = required_topics.outputTopicsAsKeyValues();

      // Add the tracker to allocated trackers + increase its reliability
      tracker->reliability_.adjustReliability();
      //allocated_trackers_[res.rmp.resource_id] = tracker;
      allocated_trackers_hack_[res.rmp.resource_id] = std::pair<TrackerInfoPtr, std::vector<int>>(tracker, sub_resource_ids);

      return;
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      // TODO: Make sure that send error add the name of the function where the error was sent
      SEND_ERROR(error_stack);
    }
  }

  throw CREATE_ERROR(temoto_core::error::Code::NO_TRACKERS_FOUND, "Could not find trackers for the requested category");
}

/*
 * Unload tracker callback
 */
void ContextManager::unloadTrackerCb(LoadTracker::Request& req, LoadTracker::Response& res)
{
  (void)req; // Suppress "unused variable" compiler warnings

  // Remove the tracker from the list of allocated trackers
  auto it = allocated_trackers_hack_.find(res.rmp.resource_id);
  if (it != allocated_trackers_hack_.end())
  {
    TEMOTO_DEBUG_STREAM("Erasing a tracker from the list of allocated trackers");
    allocated_trackers_hack_.erase(it);
  }
  else
  {
    throw CREATE_ERROR(temoto_core::error::Code::RESOURCE_NOT_FOUND, "Could not unload the tracker, because"
                       " it does not exist in the list of allocated trackers");
  }
}

/*
 * Parse trackers
 */
void ContextManager::parseTrackers(std::string config_path)
{
  // Read in the config file
  std::ifstream in(config_path);
  YAML::Node config = YAML::Load(in);

  // Check if it is a map
  if (!config.IsMap())
  {
    // TODO Throw
    std::cout << " throw throw throw \n";
    return;
  }

  // Iterate over different tracker categories (hand trackers, artag trackers, ...)
  for (YAML::const_iterator tracker_type_it = config.begin(); tracker_type_it != config.end(); ++tracker_type_it)
  {
    // Each category must contain a sequence of tracking methods
    if (!tracker_type_it->second.IsSequence())
    {
      // TODO Throw
      std::cout << " throw TODO throw TODO \n";
      return;
    }

    // Get the category of the tracker
    std::string tracker_category = tracker_type_it->first.as<std::string>();

    // Iterate over different tracking methods within the given category
    for (YAML::const_iterator method_it = tracker_type_it->second.begin();
         method_it != tracker_type_it->second.end();
         ++method_it)
    {
      try
      {
        // Convert the tracking method yaml description into TrackerInfo
        temoto_context_manager::TrackerInfo tracker_info = method_it->as<temoto_context_manager::TrackerInfo>();
        tracker_info.setType(tracker_category);

        // Add the tracking method into the map of locally known trackers
        categorized_trackers_[tracker_category].push_back(std::make_shared<temoto_context_manager::TrackerInfo>(tracker_info));

        // TODO: Print via TEMOTO_DEBUG
        // std::cout << tracker_info.toString() << std::endl;
      }
      catch (YAML::InvalidNode e)
      {
        // print out the error message
        std::cout << "Conversion failed: " << e.what() << std::endl;
      }
    }
  }
}

/*
 * Status callback 1
 */
void ContextManager::statusCb1(temoto_core::ResourceStatus& srv)
{
  (void)srv; // Suppress "unused parameter" compiler warnings
  /* TODO */
}

/*
 * Status callback 2
 */
void ContextManager::statusCb2(temoto_core::ResourceStatus& srv)
{
  TEMOTO_DEBUG("Received a status message.");

  // If local sensor failed, adjust package reliability and advertise to other managers via
  // synchronizer.
  if (srv.request.status_code == temoto_core::rmp::status_codes::FAILED)
  {
    TEMOTO_DEBUG("A resource, that a running tracker depends on, has failed");

//    auto it = allocated_trackers_.find(srv.request.resource_id);
//    if (it != allocated_trackers_.end())

    int val = srv.request.resource_id;

    auto it = std::find_if(allocated_trackers_hack_.begin(), allocated_trackers_hack_.end(),
              [val](const std::pair<int, std::pair<TrackerInfoPtr, std::vector<int>>>& pair_in)
              {
                for (const auto& client_id : pair_in.second.second)
                {
                  if (client_id == val)
                  {
                    return true;
                  }
                }
                return false;
              });

    if (it != allocated_trackers_hack_.end())
    {
      TEMOTO_INFO("Tracker of type '%s' (pipe size: %d) has stopped working",
                   (it->second.first)->getType().c_str(),
                   (it->second.first)->getPipeSize());

      // Reduce the reliability of the tracker
      (it->second.first)->reliability_.adjustReliability(0);
      detection_method_history_[(it->second.first)->getType()].adjustReliability(0);
    }
  }
}


void ContextManager::addDetectionMethod(std::string detection_method)
{
  if (detection_method_history_.find(detection_method) == detection_method_history_.end())
  {
    detection_method_history_[detection_method] = temoto_core::Reliability();
    std::cout << "Added new detection method !!!!! \n";
  }
}

void ContextManager::addDetectionMethods(std::vector<std::string> detection_methods)
{
  for (std::string& detection_method : detection_methods)
  {
    addDetectionMethod(detection_method);
  }
}

std::vector<std::string> ContextManager::getOrderedDetectionMethods()
{
  std::vector<std::pair<std::string, temoto_core::Reliability>> ordered_detection_methods;

  for ( auto it = detection_method_history_.begin()
      ; it != detection_method_history_.end()
      ; it++)
  {
    ordered_detection_methods.push_back(*it);
  }

  std::sort( ordered_detection_methods.begin()
           , ordered_detection_methods.end()
           , [](const std::pair<std::string, temoto_core::Reliability>& lhs, const std::pair<std::string, temoto_core::Reliability>& rhs)
             {
               return lhs.second.getReliability() >
                      rhs.second.getReliability();
             });

  std::vector<std::string> odm_vec;

  for (auto odm : ordered_detection_methods)
  {
    std::cout << odm.first << " --- " << odm.second.getReliability() << std::endl;
    odm_vec.push_back(odm.first);
  }

  return odm_vec;
}

}  // namespace temoto_context_manager
