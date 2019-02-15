#ifndef ENV_MODEL_REPOSITORY_H
#define ENV_MODEL_REPOSITORY_H

#include "temoto_core/common/ros_serialization.h"
#include "temoto_context_manager/NodeContainer.h"


namespace temoto_context_manager 
{
namespace emr 
{

/**
 * @brief Wrapper class to store pointers to all nodes of a tree in a map
 * 
 */
class EnvironmentModelRepository
{
public:
  std::map<std::string, std::shared_ptr<Node>> nodes;
  void advertiseAllNodes();

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
  std::shared_ptr<Node> getRootNode() {return rootNode;}
  NodePtr getNodeByName(std::string node_name)
  {
    return nodes[node_name];
  }

  EnvironmentModelRepository() {}
  ~EnvironmentModelRepository() {}
private:
  std::shared_ptr<Node>> root_node;
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
  std::weak_ptr<Node> parent;
  std::vector<std::shared_ptr<Node>> children;
  std::shared_ptr<PayloadEntry> payload;

public:

  void addChild(std::shared_ptr<Node> child);
  void setParent(std::shared_ptr<Node> parent);
  std::weak_ptr<Node> getParent()
  {
    return parent;
  }
  std::vector<std::shared_ptr<Node>> getChildren()
  {
    return children;
  }
  std::shared_ptr<PayloadEntry> getPayload()
  {
    return payload;
  }
  std::string getName() {return payload->getName();}

  // Overwrite the content of the payload pointer, while keeping the address
  void setPayload(shared_ptr<PayloadEntry> plptr) {*payload = *plptr;}

  ~Node() {}
  // Default constructor creates node with no connections and type "0"
  Node::Node() {}
  Node::Node(std::shared_ptr<PayloadEntry> payload) : payload(payload) {}
};

/**
 * @brief Abstract base class for payloads
 * 
 */
class PayloadEntry
{
private:
  
public:
  std::string type;

  PayloadEntry::PayloadEntry(std::string type) : type(type) {}

  PayloadEntry::~PayloadEntry() {}

  PayloadEntry::PayloadEntry() {}

  virtual std::string getName() = 0;
  std::string getType() {return type;}
};

template <class ROSMsg>
class ROSPayload : public PayloadEntry
{
private:
  ROSMsg payload;
public:

  std::string getName()
  {
    return payload.name;
  }
  ROSMsg getPayload() {return payload};
  ROSPayload()
  {
  }
  ROSPayload(ROSMsg payload) : payload(payload)
  {
  }

  ~ROSPayload()
  {
  }
};



} // namespace emr
} // namespace temoto_context_manager
#endif