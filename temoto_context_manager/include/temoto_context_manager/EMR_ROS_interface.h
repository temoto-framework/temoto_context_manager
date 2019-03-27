#ifndef TEMOTO_EMR_ROS_INTERFACE_H
#define TEMOTO_EMR_ROS_INTERFACE_H

#include <algorithm>


#include "temoto_context_manager/context_manager_containers.h"
#include "temoto_context_manager/env_model_repository.h"

#include "ros/package.h"
#include "temoto_core/common/ros_serialization.h"

namespace EMR_ROS_Interface
{

class EMR_ROS_interface
{
private:
    emr::EnvironmentModelRepository& env_model_repository_;
public:
    EMR_ROS_interface(emr::EnvironmentModelRepository& emr) : env_model_repository_(emr) {}

  template<class Container>
  Container getContainer(std::shared_ptr<emr::Item> itemptr)
  {
    return std::dynamic_pointer_cast<emr::ROSPayload<Container>>
      (itemptr->getPayload())
        ->getPayload();
  }

  /**
   * @brief Update the EMR structure with new information
   * 
   * @param items_to_add 
   * @param from_other_manager 
   * @return std::vector<temoto_context_manager::ItemContainer> that could not be added
   */
  std::vector<temoto_context_manager::ItemContainer> updateEMR(const std::vector<temoto_context_manager::ItemContainer> & items_to_add, bool update_time=false);

  /**
   * @brief Debug function to traverse through EMR tree 
   * 
   * @param root 
   */
  void traverseEMR(const emr::Item& root);
  
  /**
   * @brief Add or update a single item of the EMR
   * 
   * @tparam Container 
   * @param container 
   * @param container_type 
   */
  template <class Container>
  bool addOrUpdateEMRItem(const Container & container, const std::string& container_type);

  /**
   * @brief Save the EMR state as a temoto_context_manager::ItemContainer vector
   * 
   * @param emr 
   * @return std::vector<temoto_context_manager::ItemContainer> 
   */
  std::vector<temoto_context_manager::ItemContainer> EMRtoVector();

  /**
   * @brief Recursive helper function to save EMR state
   * 
   * @param currentItem 
   * @param items 
   */
  void EMRtoVectorHelper(const emr::Item& currentItem, std::vector<temoto_context_manager::ItemContainer>& items);
};

} // namespace EMR_ROS_Interface

#endif