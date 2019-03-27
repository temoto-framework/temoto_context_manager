#ifndef TEMOTO_EMR_ROS_INTERFACE_H
#define TEMOTO_EMR_ROS_INTERFACE_H

#include <algorithm>

#include "temoto_context_manager/context_manager_containers.h"
#include "temoto_context_manager/env_model_repository.h"

#include "ros/package.h"
#include "temoto_core/common/ros_serialization.h"

namespace emr_ros_interface
{

template <class RosMsg>
class RosPayload : public emr::PayloadEntry
{
private:
  RosMsg payload_;
  ros::Time last_modified;
  std::string maintainer_;
public:
  void updateTime()
  {
    last_modified = ros::Time::now();
  }
  void updateTime(ros::Time new_time)
  {
    last_modified = new_time;
  }
  ros::Time getTime() {return last_modified;}

  std::string getMaintainer() {return maintainer_;}

  void setMaintainer(const std::string& maintainer)
  {
    maintainer_ = maintainer;
  }
  /**
   * @brief Get the Name object
   * 
   * @return std::string 
   */
  const std::string& getName() const
  {
    return payload_.name;
  }
  const RosMsg& getPayload() const {return payload_;};
  /**
   * @brief Set the Payload object
   * 
   * @param payload 
   */
  void setPayload(RosMsg & payload) {payload_ = payload;};

  RosPayload(RosMsg payload) : payload_(payload)
  {
  }
  RosPayload(RosMsg payload, ros::Time last_modified, std::string maintainer) 
    : payload_(payload), last_modified(last_modified), maintainer_(maintainer)
  {
  }
  RosPayload(RosMsg payload, std::string maintainer) 
    : payload_(payload), maintainer_(maintainer)
  {
  }

};

class EmrRosInterface
{
private:
    emr::EnvironmentModelRepository& env_model_repository_;
public:
    EmrRosInterface(emr::EnvironmentModelRepository& emr) : env_model_repository_(emr) {}

  template<class Container>
  Container getContainer(const std::string name)
  {
    return getRosPayloadPtr<Container>(name)->getPayload();
  }
  template<class Container>
  std::shared_ptr<RosPayload<Container>> getRosPayloadPtr(const std::string& name)
  {
    return std::dynamic_pointer_cast<RosPayload<Container>>
      (env_model_repository_.getItemByName(name)->getPayload());
  }

  /**
   * @brief Update the EMR structure with new information
   * 
   * @param items_to_add 
   * @param from_other_manager 
   * @return std::vector<temoto_context_manager::ItemContainer> that could not be added
   */
  std::vector<temoto_context_manager::ItemContainer> updateEmr(const std::vector<temoto_context_manager::ItemContainer> & items_to_add, bool update_time=false);

  /**
   * @brief Debug function to traverse through EMR tree 
   * 
   * @param root 
   */
  void traverseEmr(const emr::Item& root);
  
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
                          const temoto_context_manager::ItemContainer& ic, 
                          const bool update_time);

  /**
   * @brief Save the EMR state as a temoto_context_manager::ItemContainer vector
   * 
   * @param emr 
   * @return std::vector<temoto_context_manager::ItemContainer> 
   */
  std::vector<temoto_context_manager::ItemContainer> EmrToVector();

  /**
   * @brief Recursive helper function to save EMR state
   * 
   * @param currentItem 
   * @param items 
   */
  void EmrToVectorHelper(const emr::Item& currentItem, std::vector<temoto_context_manager::ItemContainer>& items);
};

} // namespace emr_ros_interface

#endif