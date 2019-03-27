#ifndef TEMOTO_ENV_MODEL_REPOSITORY_H
#define TEMOTO_ENV_MODEL_REPOSITORY_H

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

template <class ROSMsg>
class ROSPayload : public PayloadEntry
{
private:
  ROSMsg payload_;
public:
  /**
   * @brief Get the Name object
   * 
   * @return std::string 
   */
  const std::string& getName() const
  {
    return payload_.name;
  }
  const ROSMsg& getPayload() const {return payload_;};
  /**
   * @brief Set the Payload object
   * 
   * @param payload 
   */
  void setPayload(ROSMsg & payload) {payload_ = payload;};

  ROSPayload(ROSMsg payload) : payload_(payload)
  {
  }

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
public:
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

};



} // namespace emr
#endif