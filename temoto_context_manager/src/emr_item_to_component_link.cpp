#include "temoto_context_manager/emr_item_to_component_link.h"

namespace temoto_context_manager
{
ComponentToEmrLink::ComponentToEmrLink( temoto_component_manager::Component component
                                      , std::shared_ptr<emr::Item> emr_item)
: component_(component)
, emr_item_(emr_item)
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
  return emr_item_->getName();
}

} // temoto_context_manager namespace