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
#include "temoto_context_manager/env_model_repository.h"

namespace ser = ros::serialization;

namespace temoto_context_manager 
{
namespace emr 
{


PayloadEntry::PayloadEntry(std::string type) : type(type) {}

PayloadEntry::~PayloadEntry() {}

PayloadEntry::PayloadEntry() : type("-1") {}


/**
 * @brief Add child node to existing node
 * 
 * @param child - pointer to the node to be added
 */
void Node::addChild(std::shared_ptr<Node> child)
{
  children.push_back(child);
  child->setParent(shared_from_this());
}
/**
 * @brief Set parent of node
 * 
 * @param parent 
 */
void Node::setParent(std::shared_ptr<Node> parent)
{
  parent = parent;
}

// Default constructor creates node with no connections and type "0"
Node::Node() : payload(PayloadEntry("0")) {}
Node::Node(PayloadEntry map_container) : payload(map_container) {}


/**
 * @brief Function to traverse and print out every node type in the tree
 * 
 * @param root 
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

/**
 * @brief Helper function to recursively find root node of tree
 * 
 * @param pNode pointer to any node in the tree
 * @return std::weak_ptr<Node> 
 */
std::weak_ptr<Node> findRoot(std::shared_ptr<Node> pNode)
{
  std::cout << pNode->getPayload().getType();
  if (pNode->getParent().expired())
  {
    return pNode;
  }
  return findRoot(pNode->getParent().lock());
}

/**
 * @brief Serialize the Object
 * 
 * @tparam Archive 
 * @param archive - output archive
 */
template<class Archive>
void ObjectEntry::save(Archive& archive) const {
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

/**
 * @brief Deserialize the object
 * 
 * 
 * @tparam Archive 
 * @param archive 
 */
template<class Archive>
void ObjectEntry::load(Archive& archive) {
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


template<class Archive>
void MapEntry::save(Archive& archive) const{
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
template<class Archive>
void MapEntry::load(Archive& archive) {
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



} // namespace emr
} // namespace temoto_context_manager
