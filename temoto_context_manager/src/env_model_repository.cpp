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



} // namespace emr
} // namespace temoto_context_manager
