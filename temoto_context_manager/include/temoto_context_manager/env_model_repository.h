#ifndef TEMOTO_ENV_MODEL_REPOSITORY_H
#define TEMOTO_ENV_MODEL_REPOSITORY_H

namespace temoto_context_manager 
{
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
 * @brief A single node in the EMR tree
 * 
 * A node contains a payload and pointers to its (singular) parent and children.
 * 
 */
class Node : public std::enable_shared_from_this<Node>
{
private:
  std::weak_ptr<Node> parent_;
  std::vector<std::shared_ptr<Node>> children_;
  std::shared_ptr<PayloadEntry> payload_;

public:

  void addChild(std::shared_ptr<Node> child);
  /**
   * @brief Set the Parent pointer
   * 
   * NB! Don't use this manually. Use addChild() of parent instead.
   * 
   * @param parent 
   */
  void setParent(std::shared_ptr<Node> parent);
  const std::weak_ptr<Node>& getParent() const
  {
    return parent_;
  }
  /**
   * @brief Get children of node
   * 
   * @return std::vector<std::shared_ptr<Node>> 
   */
  const std::vector<std::shared_ptr<Node>>& getChildren() const
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
   * @brief Check if the node is a root node
   * 
   * A node is a root node if it has no parent i.e. the weak pointer
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

  Node(std::shared_ptr<PayloadEntry> payload) : payload_(payload) {}
};

/**
 * @brief Wrapper class to store pointers to all nodes of a tree in a map
 * 
 */
class EnvironmentModelRepository
{
private:
  std::map<std::string, std::shared_ptr<Node>> nodes;
public:
  /**
   * @brief Get the root nodes of the structure
   * 
   * Since the EMR can have several disconnected trees and floating nodes,
   * we need to be able to find the root nodes to serialize the tree
   * 
   * @return std::vector<std::shared_ptr<Node>> 
   */
  std::vector<std::shared_ptr<Node>> getRootNodes() const;
  /**
   * @brief Add a node to the EMR
   * 
   * If parent name is empty, the node will be unattached and not a part of the tree.
   * You can call the node's setParent() method manually later.
   * 
   * If the parent name is not empty, make sure the corresponding parent exists.
   * 
   * The first node added to the EMR will be assigned as the root node.
   * 
   * @param parent name of parent node
   * @param name name of node to be added
   * @param entry pointer to payload
   */
  void addNode(std::string name, std::string parent, std::shared_ptr<PayloadEntry> entry);
  /**
   * @brief Update EMR node
   * 
   * @param name string name of node
   * @param entry pointer to payload
   */
  void updateNode(std::string name, std::shared_ptr<PayloadEntry> entry);
  /**
   * @brief Get shared_ptr to node by name
   * 
   * @param node_name 
   * @return std::shared_ptr<Node> 
   */
  const std::shared_ptr<Node>& getNodeByName(std::string node_name)
  {
    return nodes[node_name];
  }
  /**
   * @brief Check if EMR contains a node with the given name
   * 
   * @param name 
   * @return true if node exists
   * @return false if node does not exist
   */
  bool hasNode(std::string name);

};



} // namespace emr
} // namespace temoto_context_manager
#endif