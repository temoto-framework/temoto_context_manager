#ifndef EMR_ITEM_TO_COMPONENT_LINK__CONTEXT_MANAGER_H
#define EMR_ITEM_TO_COMPONENT_LINK__CONTEXT_MANAGER_H

#include "temoto_component_manager/Component.h"
#include "temoto_component_manager/Pipe.h"
#include "temoto_context_manager/env_model_repository.h"
#include <mutex>
#include <vector>
#include <map>

namespace temoto_context_manager
{
class ComponentToEmrLink
{
public:
  // ComponentToEmrLink( temoto_component_manager::Component component
  //                   , std::shared_ptr<emr::Item> emr_item);

  ComponentToEmrLink( temoto_component_manager::Component component
                    , std::string emr_item_name);

  std::string getComponentName() const;
  const temoto_component_manager::Component& getComponent() const;
  std::string getComponentType() const;
  std::string getEMRItemName() const;

private:
  temoto_component_manager::Component component_;
  std::string emr_item_name_;
  //std::shared_ptr<emr::Item> emr_item_;
};

typedef std::map<std::string, std::vector<temoto_component_manager::Pipe>> CategorizedPipes;
typedef std::vector<temoto_component_manager::Component> ComponentInfos;

class ComponentToPipeLink
{
  ComponentToPipeLink( temoto_component_manager::Component component
                     , temoto_component_manager::Pipe pipe_info
                     , unsigned int segment_index);

private:
  temoto_component_manager::Component component_info_;
  temoto_component_manager::Pipe pipe_info_;
  unsigned int segment_index_;
};

class ComponentToEmrRegistry
{
public:
  // const std::vector<> getComponentLinksByType() const;

  /**
   * @brief Add component to emr item link
   * 
   * @param component 
   * @param emr_item_name
   */
  void addLink(temoto_component_manager::Component component, std::string emr_item_name);

  /**
   * @brief Checks if a link exists
   * 
   * @param component_name 
   */
  bool hasLink(std::string component_name) const;

  ComponentInfos hasLinks(std::string component_type) const;

  /**
   * @brief Removes the link
   * 
   * @param component_name 
   * @return true if link was successfully removed
   * @return false if the link does not exist
   */
  bool removeLink(std::string component_name);


  bool hasPipe(std::string pipe_type) const;

  bool getPipeByType(std::string pipe_type, temoto_component_manager::Pipe& ret_pipe);

  bool addPipe(temoto_component_manager::Pipe pipe_info);

  bool setPipes(CategorizedPipes cat_pipes);

private:
  /// Mutex for protecting links from data races
  mutable std::mutex component_rw_mutex_;
  mutable std::mutex pipe_rw_mutex_;

  std::vector<ComponentToEmrLink> component_to_emr_links_;
  std::vector<ComponentToPipeLink> component_to_pipe_links_;
  CategorizedPipes categorized_pipe_infos_;

};
} // temoto_context_manager namespace

#endif