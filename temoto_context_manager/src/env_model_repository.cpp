#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <vector>
#include "temoto_context_manager/env_model_repository.h"

namespace temoto_context_manager 
{
namespace emr 
{
void EnvironmentModelRepository::addNode(std::string name, std::string parent, std::shared_ptr<PayloadEntry> payload)
{
  
  // Check if we need to attach to a parent
  if (parent == "")
  {
    // Add the node to the tree without a parent
    nodes[name] = std::make_shared<Node>(Node(payload));
  }
  else
  {
    // Parent is legit, add the node to the tree
    nodes[name] = std::make_shared<Node>(Node(payload));
    // Add the new node as a child to the parent, creating the child -> parent link in the process
    nodes[parent]->addChild(nodes[name]);
  }
}

void EnvironmentModelRepository::updateNode(std::string name, std::shared_ptr<PayloadEntry> plptr)
{
  nodes[name]->setPayload(plptr);
}

bool EnvironmentModelRepository::hasNode(std::string name)
{
  return nodes.count(name);
}
std::vector<std::shared_ptr<Node>> EnvironmentModelRepository::getRootNodes() const
{
  std::vector<std::shared_ptr<Node>> root_nodes;
  for (auto const& pair : nodes)
  {
    if (pair.second->isRoot())
    {
      root_nodes.push_back(pair.second);
    }
  }
  return root_nodes;
}
/**
 * @brief Add child node to existing node
 * 
 * @param child - pointer to the node to be added
 */
void Node::addChild(std::shared_ptr<Node> child)
{
  children_.push_back(child);
  child->setParent(shared_from_this());
}
/**
 * @brief Set parent of node
 * 
 * @param parent 
 */
void Node::setParent(std::shared_ptr<Node> parent)
{
//   TEMOTO_DEBUG("Attempting to add " << parent->name.c_str() << " as parent to " << name);
  // Make sure the node does not already have a parent
  if (parent_.expired()) {
    parent_ = parent;
  }
  else
  {
    // TEMOTO_ERROR("Node already has a parent.")
  } 
}

} // namespace emr
} // namespace temoto_context_manager
