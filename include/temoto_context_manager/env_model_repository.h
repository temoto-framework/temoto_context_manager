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

#ifndef TEMOTO_CONTEXT_MANAGER__ENV_MODEL_REPOSITORY_H
#define TEMOTO_CONTEXT_MANAGER__ENV_MODEL_REPOSITORY_H

#include <mutex>
namespace emr 
{

/**
 * @brief Abstract base class for payloads
 * 
 */
class PayloadEntry
{
protected:
  std::string type;
public:

  PayloadEntry(std::string type) : type(type) {}

  ~PayloadEntry() {}

  PayloadEntry() {}

  const virtual std::string& getName() const = 0;
  /**
   * @brief Get the type of the Payload
   * 
   * @return std::string
   */
  const std::string& getType() const {return type;}
  void setType(std::string ntype) {type = ntype;}
};


/**
 * @brief A single item in the EMR tree
 * 
 * A item contains a payload and pointers to its (singular) parent and children.
 * 
 */
class Item : public std::enable_shared_from_this<Item>
{
private:
  std::weak_ptr<Item> parent_;
  std::vector<std::shared_ptr<Item>> children_;
  std::shared_ptr<PayloadEntry> payload_;

public:

  void addChild(std::shared_ptr<Item> child);
  /**
   * @brief Set the Parent pointer
   * 
   * NB! Don't use this manually. Use addChild() of parent instead.
   * 
   * @param parent 
   */
  void setParent(std::shared_ptr<Item> parent);
  const std::weak_ptr<Item>& getParent() const
  {
    return parent_;
  }
  /**
   * @brief Get children of item
   * 
   * @return std::vector<std::shared_ptr<Item>> 
   */
  const std::vector<std::shared_ptr<Item>>& getChildren() const
  {
    return children_;
  }

  std::vector<std::shared_ptr<Item>>* getChildrenNonConst() 
  {
    return &children_;
  }

  /**
   * @brief Get the pointer to Payload
   * 
   * @return std::shared_ptr<PayloadEntry> 
   */
  const std::shared_ptr<PayloadEntry>& getPayload() const
  {
    return payload_;
  }
  const std::string& getName() const {return payload_->getName();}
  
  /**
   * @brief Check if the item is a root item
   * 
   * A item is a root item if it has no parent i.e. the weak pointer
   * to the parent is expired.
   * 
   * @return true 
   * @return false
   */
  bool isRoot() {return parent_.expired();}
  /**
   * @brief Set the Payload
   * 
   * @param plptr shared_ptr to Object inheriting PayloadEntry
   */
  void setPayload(std::shared_ptr<PayloadEntry> plptr) {payload_ = plptr;}

  Item(std::shared_ptr<PayloadEntry> payload) : payload_(payload) {}
};

/**
 * @brief Wrapper class to store pointers to all items of a tree in a map
 * 
 */
class EnvironmentModelRepository
{
private:
  std::map<std::string, std::shared_ptr<Item>> items;
  mutable std::mutex emr_mutex; 
public:
  const std::map<std::string, std::shared_ptr<Item>>& getItems() 
  {
    std::lock_guard<std::mutex> lock(emr_mutex);
    return items;
  }
  /**
   * @brief Get the root items of the structure
   * 
   * Since the EMR can have several disconnected trees and floating items,
   * we need to be able to find the root items to serialize the tree
   * 
   * @return std::vector<std::shared_ptr<Item>> 
   */
  std::vector<std::shared_ptr<Item>> getRootItems() const;
  /**
   * @brief Add a item to the EMR
   * 
   * If parent name is empty, the item will be unattached and not a part of the tree.
   * You can call the item's setParent() method manually later.
   * 
   * If the parent name is not empty, make sure the corresponding parent exists.
   * 
   * The first item added to the EMR will be assigned as the root item.
   * 
   * @param parent name of parent item
   * @param name name of item to be added
   * @param entry pointer to payload
   */
  void addItem(const std::string& name, const std::string& parent, std::shared_ptr<PayloadEntry> payload);
  /**
   * @brief Update EMR item
   * 
   * @param name string name of item
   * @param entry pointer to payload
   */
  void updateItem(const std::string& name, std::shared_ptr<PayloadEntry> entry);
  /**
   * @brief Get shared_ptr to item by name
   * 
   * @param item_name 
   * @return std::shared_ptr<Item> 
   */
  const std::shared_ptr<Item> getItemByName(std::string item_name)
  {
    std::lock_guard<std::mutex> lock(emr_mutex);
    return items[item_name];
  }
  /**
   * @brief Check if EMR contains a item with the given name
   * 
   * @param name 
   * @return true if item exists
   * @return false if item does not exist
   */
  bool hasItem(const std::string& name);
  void removeItem(const std::string& name);
};

} // namespace emr
#endif