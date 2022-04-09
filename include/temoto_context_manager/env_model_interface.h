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

#ifndef TEMOTO_CONTEXT_MANAGER__ENV_MODEL_INTERFACE_H
#define TEMOTO_CONTEXT_MANAGER__ENV_MODEL_INTERFACE_H

#include "temoto_context_manager/context_manager_containers.h"
#include <ros/ros.h>

namespace temoto_context_manager
{

/**
 * @brief Abstract class to describe required functionalities of environment model 
 * 
 */
class EnvModelInterface
{
public:
/**
 * @brief Get the type of EM item by name
 * 
 * @param name 
 * @return std::string 
 */
  virtual std::string getTypeByName(const std::string& name) = 0;

  // C++ does not support templated virtual classes :(
  /**
   * @brief Get an object type item
   * 
   * @param name 
   * @return ObjectContainer 
   */
  virtual ObjectContainer getObject(const std::string& name) = 0;
  /**
   * @brief Get a map type item
   * 
   * @param name 
   * @return MapContainer 
   */
  virtual MapContainer getMap(const std::string& name) = 0;
  /**
   * @brief Get a Component type item
   * 
   * @param name 
   * @return ComponentContainer 
   */
  virtual ComponentContainer getComponent(const std::string& name) = 0;
  /**
   * @brief Get a Robot type item
   * 
   * @param name 
   * @return RobotContainer 
   */
  virtual RobotContainer getRobot(const std::string& name) = 0;
  /**
   * @brief Get first parent item of type MAP
   * 
   * @param name 
   * @return MapContainer 
   */
  virtual MapContainer getNearestParentMap(const std::string& name) = 0;
  /**
   * @brief Get first parent item of type OBJECT
   * 
   * @param name 
   * @return ObjectContainer 
   */
  virtual ObjectContainer getNearestParentObject(const std::string& name) = 0;
  /**
   * @brief Get first parent item of type COMPONENT
   * 
   * @param name 
   * @return ComponentContainer 
   */
  virtual ComponentContainer getNearestParentComponent(const std::string& name) = 0;
  /**
   * @brief Get first parent item of type ROBOT
   * 
   * @param name 
   * @return RobotContainer 
   */
  virtual RobotContainer getNearestParentRobot(const std::string& name) = 0;

  /**
   * @brief Check if the EM has an item with this name
   * 
   * @param name 
   * @return true 
   * @return false 
   */
  virtual bool hasItem(const std::string& name) = 0;
  /**
   * @brief Remove an item
   * 
   * @param name 
   */
  virtual void removeItem(const std::string& name) = 0;
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
   * @brief Save the EM state as a temoto_context_manager::ItemContainer vector
   * 
   * @return std::vector<temoto_context_manager::ItemContainer> 
   */
  virtual std::vector<ItemContainer> EmrToVector() = 0;

  /**
   * @brief Update pose of EM item
   * 
   * @tparam Container 
   * @param name 
   * @param newPose 
   */
  virtual void updatePose(const std::string& name, const geometry_msgs::PoseStamped& newPose) = 0;
};

} // namespace emr_ros_interface

#endif