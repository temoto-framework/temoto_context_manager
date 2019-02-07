#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <vector>
#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/types/vector.hpp"
#include <ros/serialization.h>
#include "temoto_context_manager/context_manager_containers.h"

namespace ser = ros::serialization;

class PayloadEntry
{
private:
  std::string type;

public:
  PayloadEntry(std::string type) : type(type)
  {
  }

  ~PayloadEntry()
  {
  }

  PayloadEntry() : type("-1")
  {
  }
  std::string getType()
  {
    return type;
  }
  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(type);
  }
};

class Node : public std::enable_shared_from_this<Node>
{
private:
  std::weak_ptr<Node> parent;
  std::vector<std::shared_ptr<Node> > children;
  PayloadEntry payload;

public:
  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(parent, children, payload);
  }
  void addChild(std::shared_ptr<Node> child)
  {
    children.push_back(child);
    child->setParent(shared_from_this());
  }
  void setParent(std::shared_ptr<Node> parent)
  {
    parent = parent;
  }
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

  Node() : payload(PayloadEntry("0"))
  {
  }
  Node(PayloadEntry map_container) : payload(map_container)
  {
  }
  ~Node()
  {
  }
};

/**
 * Function to recursively visit every node in a Tree starting from the root
 *
 */
void traverseTree(Node root)
{
  std::cout << root.getPayload().getType();
  std::vector<std::shared_ptr<Node> > children = root.getChildren();
  for (uint32_t i = 0; i < children.size(); i++)
  {
    traverseTree(*children[i]);
  }
}
std::weak_ptr<Node> findRoot(std::shared_ptr<Node> pNode)
{
  std::cout << pNode->getPayload().getType();
  if (pNode->getParent().expired())
  {
    return pNode;
  }
  return findRoot(pNode->getParent().lock());
}

class Tree
{
private:
public:
  std::map<std::string, std::shared_ptr<Node> > nodes;

  template <class Archive>
  void serialize(Archive & archive)
  {
    archive(nodes);
  }

  Tree()
  {
  }
  ~Tree()
  {
  }
};

// int main()
// {
//   std::stringstream ss;
//   {
//     cereal::JSONOutputArchive oarchive(ss);

//     std::shared_ptr<Node> root = std::make_shared<Node>(Node());
//     std::map<std::string, std::shared_ptr<Node>> nodes;
//     Tree tree = Tree();
//     tree.nodes["0"] = root;
//     tree.nodes["1"] = std::make_shared<Node>(Node(PayloadEntry("1")));
//     tree.nodes["2"] = std::make_shared<Node>(Node(PayloadEntry("2")));
//     tree.nodes["3"] = std::make_shared<Node>(Node(PayloadEntry("3")));
//     root->addChild(tree.nodes["1"]);
//     tree.nodes["1"]->addChild(tree.nodes["2"]);
//     tree.nodes["2"]->addChild(tree.nodes["3"]);

//     // traverseTree(*root);
//     // std::cout << findRoot(nodes["3"]).lock()->getPayload().getType();
//     oarchive(tree);
//   }
//   // std::cout << ss.str();
//   {
//     Tree newTree = Tree();

//     cereal::JSONInputArchive iarchive(ss);
//     iarchive(newTree);
//     traverseTree(*newTree.nodes["0"]);
//     std::cout << findRoot(newTree.nodes["3"]).lock()->getPayload().getType();
//   }
//   // std::cout << "\n" << nodes["3"]->getParent().lock()->getPayload().getType();
// }

class ObjectEntry : public PayloadEntry
{
private:
  temoto_context_manager::ObjectContainer object_container;
  mutable std::vector<uint8_t> payload_byte_array;
public:

  template <class Archive>
  void save(Archive& archive) const {
    // Serialize the object_container
    uint32_t payload_size = ros::serialization::serializationLength(object_container);
    boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);
    ser::OStream stream(buffer.get(), payload_size);
    ser::serialize(stream, object_container);

    // Fill out the byte array
    for (uint32_t i=0; i<payload_size; i++)
    {
      payload_byte_array.push_back(buffer.get()[i]);
    }
    archive(payload_byte_array);
  }
  
  template <class Archive>
  void load(Archive& archive) {
    archive(payload_byte_array);
    uint32_t payload_size = payload_byte_array.size();
    boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);

    // Fill buffer with the serialized payload
    for (uint32_t i=0; i<payload_size; i++)
    {
      (buffer.get())[i] = payload_byte_array[i];
    }

    // Convert the serialized payload to msg
    ser::IStream stream(buffer.get(), payload_size);
    ser::deserialize(stream, object_container);
  }
  ObjectEntry()
  {
  }
  ~ObjectEntry()
  {
  }
};

// TODO: 
// Uncomment once the MapContainer actually exists
class MapEntry : public PayloadEntry
{
private:
  temoto_context_manager::MapContainer map_container;
  mutable std::vector<uint8_t> payload_byte_array;
public:
  template <class Archive>
  void save(Archive& archive) const{
    // Serialize the map_container
    uint32_t payload_size = ros::serialization::serializationLength(map_container);
    boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);
    ser::OStream stream(buffer.get(), payload_size);
    ser::serialize(stream, map_container);

    // Fill out the byte array
    for (uint32_t i=0; i<payload_size; i++)
    {
      payload_byte_array.push_back(buffer.get()[i]);
    }
    archive(payload_byte_array);
  }

  template <class Archive>
  void load(Archive& archive) {
    archive(payload_byte_array);
    uint32_t payload_size = payload_byte_array.size();
    boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);

    // Fill buffer with the serialized payload
    for (uint32_t i=0; i<payload_size; i++)
    {
      (buffer.get())[i] = payload_byte_array[i];
    }

    // Convert the serialized payload to msg
    ser::IStream stream(buffer.get(), payload_size);
    ser::deserialize(stream, map_container);
  }

  MapEntry()
  {
  }
  ~MapEntry()
  {
  }
};


