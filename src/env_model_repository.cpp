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

void EnvironmentModelRepository::removeItem(const std::string& name)
{
  std::shared_ptr<Item> to_be_erased = items[name];
  // Remove the reference to the "item_to_be_erased" from its parent item
  if(!to_be_erased->isRoot())
  {
    auto parent_of_to_be_erased = to_be_erased->getParent();
    std::shared_ptr<Item> parent_ptr = parent_of_to_be_erased.lock();
    auto it = std::remove_if(parent_ptr->getChildrenNonConst()->begin()
    , parent_ptr->getChildrenNonConst()->end()
    , [name] (const std::shared_ptr<Item>& item)
      {
        return (item->getName() == name);
      });

    //Actually remove the elements from the children_ vector
    while (it != parent_ptr->getChildrenNonConst()->end())
    {
      it = parent_ptr->getChildrenNonConst()->erase(it);
    }
  }
  // Convert the children into root nodes
  for (auto child : to_be_erased->getChildren())
  {      
    child->setParent(std::shared_ptr<Item>(nullptr));
  }
  items.erase(name);
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
  // Make sure the item does not already have a parent
  if (parent_.expired()) 
  {
    parent_ = parent;
  }
}

} // namespace emr
