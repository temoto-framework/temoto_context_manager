#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <vector>
#include "temoto_context_manager/env_model_repository.h"

namespace emr 
{
void EnvironmentModelRepository::addItem(const std::string& name, const std::string& parent, std::shared_ptr<PayloadEntry> payload)
{
  std::lock_guard<std::mutex> lock(emr_mutex);
  // Check if we need to attach to a parent
  if (parent == "")
  {
    // Add the item to the tree without a parent
    items[name] = std::make_shared<Item>(Item(payload));
  }
  else
  {
    // Parent is legit, add the item to the tree
    items[name] = std::make_shared<Item>(Item(payload));
    // Add the new item as a child to the parent, creating the child -> parent link in the process
    items[parent]->addChild(items[name]);
  }
}

void EnvironmentModelRepository::updateItem(const std::string& name, std::shared_ptr<PayloadEntry> plptr)
{
  std::lock_guard<std::mutex> lock(emr_mutex);
  items[name]->setPayload(plptr);
}

bool EnvironmentModelRepository::hasItem(const std::string& name)
{
  std::lock_guard<std::mutex> lock(emr_mutex);
  return items.count(name);
}
std::vector<std::shared_ptr<Item>> EnvironmentModelRepository::getRootItems() const
{
  std::lock_guard<std::mutex> lock(emr_mutex);
  std::vector<std::shared_ptr<Item>> root_items;
  for (auto const& pair : items)
  {
    if (pair.second->isRoot())
    {
      root_items.push_back(pair.second);
    }
  }
  return root_items;
}
/**
 * @brief Add child item to existing item
 * 
 * @param child - pointer to the item to be added
 */
void Item::addChild(std::shared_ptr<Item> child)
{
  children_.push_back(child);
  child->setParent(shared_from_this());
}
/**
 * @brief Set parent of item
 * 
 * @param parent 
 */
void Item::setParent(std::shared_ptr<Item> parent)
{
  //   TEMOTO_DEBUG("Attempting to add " << parent->name.c_str() << " as parent to " << name);
  // Make sure the item does not already have a parent
  if (parent_.expired()) 
  {
    parent_ = parent;
  }
  else
  {
    // TEMOTO_ERROR("Item already has a parent.")
  } 
}

} // namespace emr
