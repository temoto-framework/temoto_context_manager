#ifndef ENV_MODEL_REPOSITORY_H
#define ENV_MODEL_REPOSITORY_H

#include "temoto_core/ros_serialization.h"

namespace cereal
{
  template <class Archive> 
  struct specialize<Archive, ROSPayload, cereal::specialization::member_load_save> {};
  template <class Archive> 
  struct specialize<Archive, MapEntry, cereal::specialization::member_serialize> {};
  template <class Archive> 
  struct specialize<Archive, ObjectEntry, cereal::specialization::member_serialize> {};
  // cereal no longer has any ambiguity when serializing inherited classes
}

namespace temoto_context_manager 
{
namespace emr 
{


class PayloadEntry
{
private:
  
public:
  std::string type;

  PayloadEntry(std::string type);

  ~PayloadEntry();

  PayloadEntry();

  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(type);
  }
  std::string getName() = 0;
};

template <class ROSMsg>
class ROSPayload : public PayloadEntry
{
private:
  ROSMsg payload;
  mutable std::vector<uint8_t> payload_byte_array;
public:

  template<class Archive>
  void save(Archive& archive) const{
    payload_byte_array = temoto_core::serializeROSmsg<ROSMsg>(payload);
    archive(cereal::base_class<PayloadEntry>(this), payload_byte_array);
  }
  template<class Archive>
  void load(Archive& archive) {
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

class Node : public std::enable_shared_from_this<Node>
{
private:
  std::string name;
  std::weak_ptr<Node> parent;
  std::vector<std::shared_ptr<Node> > children;
  PayloadEntry payload;

public:
  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(name, parent, children, payload);
  }
  void addChild(std::shared_ptr<Node> child);
  void setParent(std::shared_ptr<Node> parent);
  std::weak_ptr<Node> getParent()
  {
    return parent;
  }
  std::vector<std::shared_ptr<Node> > getChildren()
  {
    return children;
  }
  PayloadEntry getPayload()
  {
    return payload;
  }

  Node();
  Node(PayloadEntry payload);
  ~Node() {}
};

/**
 * @brief Wrapper class to store pointers to all nodes of a tree in a map
 * 
 */
class EnvironmentModelRepository
{
public:
  std::map<std::string, std::shared_ptr<Node> > nodes;

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
  void addNode(std::string parent, PayloadEntry entry);

  ObjectPtr findObject(std::string object_name);

  NodePtr findNodeByName(std::string node_name)
  {
    return nodes[node_name];
  }

  EnvironmentModelRepository() {}
  ~EnvironmentModelRepository() {}
};

class ObjectEntry : private ROSPayload<ObjectContainer>
{
public:

  ObjectEntry() {}
  ObjectEntry(ObjectContainer obj_container) : ROSPayload<ObjectContainer>(obj_container) {}
  ~ObjectEntry() {}

  template <class Archive>
  void serialize(Archive & archive)
  {
    archive(cereal::base_class<ROSPayload>(this));
  }
};

class MapEntry : private ROSPayload<MapContainer>
{
public:

  MapEntry() {}
  MapEntry(MapContainer map_container) : ROSPayload<MapContainer>(map_container) {}
  ~MapEntry() {}

  template <class Archive>
  void serialize(Archive & archive)
  {
    archive(cereal::base_class<ROSPayload>(this));
  }
  
};



} // namespace emr
} // namespace temoto_context_manager
#endif