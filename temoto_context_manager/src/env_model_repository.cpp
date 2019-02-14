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

void EnvironmentModelRepository::addNode(std::string parent, std::unique_ptr<PayloadEntry> payload)
{
  // Check if the parent exists
  if (nodes[parent].expired()) {
    TEMOTO_ERROR("No parent with name " << parent << " found in EMR!");
    return;
  }
  // Add the new node as a child to the parent, creating the child -> parent link in the process
  nodes[parent]->addChild(std::make_shared<Node>(Node(payload)));
}
void EnvironmentModelRepository::updateNode(std::string name, std::unique_ptr<PayloadEntry> plptr)
{
  nodes[name].updatePayload(plptr);
}
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
  TEMOTO_DEBUG("Attempting to add " << parent->name << " as parent to " << name);
  // Check if the node already has a parent
  if (parent.expired()) {
    parent = parent;
  }
  else
  {
    TEMOTO_ERROR("Node already has a parent.")
  } 
}

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
