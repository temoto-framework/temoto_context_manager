#ifndef EMR_ITEM_TO_COMPONENT_LINK__CONTEXT_MANAGER_H
#define EMR_ITEM_TO_COMPONENT_LINK__CONTEXT_MANAGER_H

#include "temoto_component_manager/Component.h"
#include "temoto_context_manager/env_model_repository.h"
#include <mutex>
#include <vector>

namespace temoto_context_manager
{
class ComponentToEmrLink
{
public:
  ComponentToEmrLink( temoto_component_manager::Component component
                    , std::shared_ptr<emr::Item> emr_item);

  std::string getComponentName() const;
  const temoto_component_manager::Component& getComponent() const;
  std::string getEMRItemName() const;

private:
  temoto_component_manager::Component component_;
  std::shared_ptr<emr::Item> emr_item_;
};

class ComponentToEmrRegistry
{
public:
  // const std::vector<> getComponentLinksByType() const;

private:
  /// Mutex for protecting links from data races
  mutable std::mutex read_write_mutex_;

  std::vector<ComponentToEmrLink> component_to_emr_links_;
};
} // temoto_context_manager namespace

#endif