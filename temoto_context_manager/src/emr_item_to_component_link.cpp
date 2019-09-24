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

#include "temoto_context_manager/emr_item_to_component_link.h"

namespace temoto_context_manager
{
ComponentToEmrLink::ComponentToEmrLink( temoto_component_manager::Component component
                                      , std::string emr_item_name)
: component_(component)
, emr_item_name_(emr_item_name)
{}

std::string ComponentToEmrLink::getComponentName() const
{
  return component_.component_name;
}

std::string ComponentToEmrLink::getComponentType() const
{
  return component_.component_type;
}

const temoto_component_manager::Component& ComponentToEmrLink::getComponent() const
{
  return component_;
}

std::string ComponentToEmrLink::getEMRItemName() const
{
  return emr_item_name_;
}


void ComponentToEmrRegistry::addLink( temoto_component_manager::Component component
                                    , std::string emr_item_name)
{
  std::lock_guard<std::mutex> guard(component_rw_mutex_); // Lock the mutex
  component_to_emr_links_.emplace_back(component, emr_item_name);
}

bool ComponentToEmrRegistry::hasLink( std::string component_name) const
{
  std::lock_guard<std::mutex> guard(component_rw_mutex_); // Lock the mutex
  for (const auto& link : component_to_emr_links_)
  {
    if (link.getComponentName() == component_name)
    {
      return true;
      break;
    }
  }
  return false;
}

ComponentInfos ComponentToEmrRegistry::hasLinks(std::string component_type) const
{
  std::lock_guard<std::mutex> guard(component_rw_mutex_); // Lock the mutex
  ComponentInfos ret_component_infos;
  for (const auto& link : component_to_emr_links_)
  {
    if (link.getComponentType() == component_type)
    {
      ret_component_infos.push_back(link.getComponent());
    }
  }
  return ret_component_infos;
}

bool ComponentToEmrRegistry::removeLink(std::string component_name)
{
  std::lock_guard<std::mutex> guard(component_rw_mutex_); // Lock the mutex
  for (auto it=component_to_emr_links_.begin();
       it!=component_to_emr_links_.end();
       it++)
  {
    if (it->getComponentName() == component_name)
    {
      component_to_emr_links_.erase(it);
      return true;
    }
  }
  return false;
}

bool ComponentToEmrRegistry::addPipe(temoto_component_manager::Pipe pipe_info)
{
  std::lock_guard<std::mutex> guard(pipe_rw_mutex_); // Lock the mutex
  categorized_pipe_infos_[pipe_info.pipe_type].push_back(pipe_info);
  return true;
}

bool ComponentToEmrRegistry::hasPipe(std::string pipe_type) const
{
  std::lock_guard<std::mutex> guard(pipe_rw_mutex_); // Lock the mutex
  if (categorized_pipe_infos_.find(pipe_type) != categorized_pipe_infos_.end())
  {
    return true;
  }
  return false;
}

bool ComponentToEmrRegistry::getPipeByType(std::string pipe_type, temoto_component_manager::Pipe& ret_pipe)
{
  std::lock_guard<std::mutex> guard(pipe_rw_mutex_); // Lock the mutex
  if (categorized_pipe_infos_.find(pipe_type) != categorized_pipe_infos_.end())
  {
    // TODO: Currently the first pipe in the vector is returned. Should be returned based on
    // reliability value
    ret_pipe = categorized_pipe_infos_[pipe_type][0];
    return true;
  }
  return false;
}

bool ComponentToEmrRegistry::setPipes(CategorizedPipes cat_pipes)
{
  std::lock_guard<std::mutex> guard(pipe_rw_mutex_); // Lock the mutex
  categorized_pipe_infos_.clear();
  categorized_pipe_infos_ = cat_pipes;
  return true;
}

} // temoto_context_manager namespace