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
  std::lock_guard<std::mutex> guard(read_write_mutex_); // Lock the mutex
  component_to_emr_links_.emplace_back(component, emr_item_name);
}

bool ComponentToEmrRegistry::hasLink( std::string component_name) const
{
  std::lock_guard<std::mutex> guard(read_write_mutex_); // Lock the mutex
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

bool ComponentToEmrRegistry::removeLink(std::string component_name)
{
  std::lock_guard<std::mutex> guard(read_write_mutex_); // Lock the mutex
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

} // temoto_context_manager namespace