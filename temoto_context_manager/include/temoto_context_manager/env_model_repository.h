#ifndef ENV_MODEL_REPOSITORY_H
#define ENV_MODEL_REPOSITORY_H

#include "temoto_core/ros_serialization.h"

namespace cereal
{
  template <class Archive> 
  struct specialize<Archive, ROSPayload, cereal::specialization::member_load_save> {};
  template <class Archive>
  struct specialize<Archive, PayloadEntry, cereal::specialization::member_serialize> {};
  // cereal no longer has any ambiguity when serializing inherited classes
}

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

  template <class Archive>
  void serialize(Archive & archive)
  {
    archive(nodes);
  }

  void advertiseAllNodes();

  /**
   * @brief Add a node to the EMR
   * 
   * If parent name is empty, the EMR root node is set as the parent.
   * 
   * @param parent name of parent node
   * @param entry payload
   */
  void addNode(std::string parent, std::unique_ptr<PayloadEntry> entry);
  void updateNode(std::string parent, std::unique_ptr<PayloadEntry> entry);

  NodePtr getNodeByName(std::string node_name)
  {
    return nodes[node_name];
  }

  EnvironmentModelRepository() {}
  ~EnvironmentModelRepository() {}
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
  std::unique_ptr<PayloadEntry> payload;

public:
  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(parent, children, payload);
  }
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
  std::unique_ptr<PayloadEntry> getPayload()
  {
    return payload;
  }
  std::string getName() {return payload->getName();}

  void setPayload(unique_ptr<PayloadEntry> plptr) {payload = plptr;}

  ~Node() {}
  // Default constructor creates node with no connections and type "0"
  Node::Node() {}
  Node::Node(std::unique_ptr<PayloadEntry> payload) : payload(payload) {}
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

  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(type);
  }
  virtual std::string getName() = 0;
  std::string getType() {return type;}
};

template <class ROSMsg>
class ROSPayload : public PayloadEntry
{
private:
  ROSMsg payload;
  mutable std::vector<uint8_t> payload_byte_array;
public:

  template<class Archive>
  void save(Archive& archive) const
  {
    payload_byte_array = temoto_core::serializeROSmsg<ROSMsg>(payload);
    archive(cereal::base_class<PayloadEntry>(this), payload_byte_array);
  }
  template<class Archive>
  void load(Archive& archive) 
  {
    archive(cereal::base_class<PayloadEntry>(this), payload_byte_array);
    payload = temoto_core::deserializeROSmsg<ROSMsg>(payload_byte_array);
  }
  std::string getName()
  {
    return payload.name;
  }
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