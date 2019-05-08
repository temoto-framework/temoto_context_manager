#ifndef TEMOTO_ENV_MODEL_INTERFACE_H
#define TEMOTO_ENV_MODEL_INTERFACE_H

#include "temoto_context_manager/context_manager_containers.h"
#include <ros/ros.h>

namespace temoto_context_manager
{

/**
 * @brief Abstract class to highlight which functionalities 
 * 
 */
class EnvModelInterface
{
public:
  virtual std::string getTypeByName(const std::string& name) = 0;

  // C++ does not support templated virtual classes :(
  virtual ObjectContainer getObject(const std::string& name) = 0;
  virtual MapContainer getMap(const std::string& name) = 0;
  virtual ComponentContainer getComponent(const std::string& name) = 0;
  virtual RobotContainer getRobot(const std::string& name) = 0;

  virtual ObjectContainer getNearestParentObject(const std::string& name) = 0;
  virtual MapContainer getNearestParentMap(const std::string& name) = 0;
  virtual ComponentContainer getNearestParentComponent(const std::string& name) = 0;
  virtual RobotContainer getNearestParentRobot(const std::string& name) = 0;

  virtual bool hasItem(const std::string& name) = 0;
  /**
   * @brief Update the EMR structure with new information
   * 
   * @param items_to_add 
   * @param from_other_manager 
   * @return std::vector<temoto_context_manager::ItemContainer> that could not be added
   */
  virtual std::vector<ItemContainer> updateEmr(const std::vector<ItemContainer> & items_to_add, bool update_time=false) = 0;
  virtual std::vector<ItemContainer> updateEmr(const ItemContainer & item_to_add, bool update_time=false) = 0;
  
  /**
   * @brief Save the EMR state as a temoto_context_manager::ItemContainer vector
   * 
   * @param emr 
   * @return std::vector<temoto_context_manager::ItemContainer> 
   */
  virtual std::vector<ItemContainer> EmrToVector() = 0;

  /**
   * @brief Update pose of EMR item
   * 
   * @tparam Container 
   * @param name 
   * @param newPose 
   */
  virtual void updatePose(const std::string& name, const geometry_msgs::PoseStamped& newPose) = 0;
};

} // namespace emr_ros_interface

#endif