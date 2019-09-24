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

/* Author: Robert Valner */

#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_CONTAINERS_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_CONTAINERS_H

#include "temoto_context_manager/ObjectContainer.h"
#include "temoto_context_manager/MapContainer.h"
#include "temoto_context_manager/ItemContainer.h" 
#include "temoto_context_manager/ComponentContainer.h"
#include "temoto_context_manager/RobotContainer.h"
#include "temoto_core/common/topic_container.h"
#include "temoto_context_manager/env_model_repository.h"

namespace emr_ros_interface
{
  namespace emr_containers
  {
    const std::string OBJECT = "OBJECT";
    const std::string MAP = "MAP";
    const std::string ROBOT = "ROBOT";
    const std::string COMPONENT = "COMPONENT";
  } // emr_containers namespace
} // emr_ros_interface namespace

// TODO: this namespace should be renamed to "emr_ros_interface"
namespace temoto_context_manager
{

// Define the type of the data that is going to be synchronized
typedef std::vector<ObjectContainer> Objects;
typedef std::shared_ptr<ObjectContainer> ObjectPtr;
typedef std::vector<ObjectPtr> ObjectPtrs;
typedef std::vector<ItemContainer> Items;
typedef std::shared_ptr<emr::Item> ItemPtr;



// ObjectContainer comparison operator
bool operator==(const ObjectContainer& ob1, const ObjectContainer& ob2);

} // temoto_context_manager namespace

#endif
