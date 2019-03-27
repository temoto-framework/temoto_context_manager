#include "temoto_context_manager/EMR_ROS_interface.h"

namespace EMR_ROS_Interface
{
std::vector<temoto_context_manager::ItemContainer> EMR_ROS_interface::updateEMR(
                  const std::vector<temoto_context_manager::ItemContainer>& items_to_add, 
                  bool update_time)
{
  
  // Keep track of failed add/update attempts
  std::vector<temoto_context_manager::ItemContainer> failed_items;
  for (const auto& item_container : items_to_add)
  {
    if (item_container.type == "OBJECT") 
    {
      // Deserialize into an temoto_context_manager::ObjectContainer object and add to EMR
      temoto_context_manager::ObjectContainer oc = 
        temoto_core::deserializeROSmsg<temoto_context_manager::ObjectContainer>(item_container.serialized_container);
      if (update_time) oc.last_modified = ros::Time::now();
      if (!addOrUpdateEMRItem(oc, "OBJECT")) failed_items.push_back(item_container);
    }
    else if (item_container.type == "MAP") 
    {
      std::cout << "Map is being added" << std::endl;
      // Deserialize into an temoto_context_manager::MapContainer object and add to EMR
      temoto_context_manager::MapContainer mc = 
        temoto_core::deserializeROSmsg<temoto_context_manager::MapContainer>(item_container.serialized_container);
      if (update_time) mc.last_modified = ros::Time::now();
      if (!addOrUpdateEMRItem(mc, "MAP")) failed_items.push_back(item_container);
    }
    else if (item_container.type == "COMPONENT") 
    {
      // Deserialize into an temoto_context_manager::ComponentContainer object and add to EMR
      temoto_context_manager::ComponentContainer cc = 
        temoto_core::deserializeROSmsg<temoto_context_manager::ComponentContainer>(item_container.serialized_container);
      if (update_time) cc.last_modified = ros::Time::now();
      if (!addOrUpdateEMRItem(cc, "COMPONENT")) failed_items.push_back(item_container);
    }
    else
    {
      // TEMOTO_ERROR_STREAM("Wrong type " << item_container.type.c_str() << "specified for EMR item");
      failed_items.push_back(item_container);
    }
  }

  return failed_items;
}

template <class Container>
bool EMR_ROS_interface::addOrUpdateEMRItem(const Container& container, const std::string& container_type)
{
  emr::ROSPayload<Container> rospl = emr::ROSPayload<Container>(container);
  rospl.setType(container_type);
  std::string name = container.name;
  std::string parent = container.parent;

  // Check for empty name field
  // Move these to the context manager interface maybe? TBD
  if (name == "") 
  {
    // TEMOTO_ERROR_STREAM("Empty string not allowed as EMR item name!");
    return false;
  }
  // Check if the parent exists
  if ((!parent.empty()) && (!env_model_repository_.hasItem(parent))) 
  {
    // TEMOTO_ERROR_STREAM("No parent with name " << parent << " found in EMR!");
    return false;
  }
  
  // Check if the object has to be added or updated
  if (!env_model_repository_.hasItem(name)) 
  {
    // Add the new item
    std::shared_ptr<emr::ROSPayload<Container>> plptr = std::make_shared<emr::ROSPayload<Container>>(rospl);
    env_model_repository_.addItem(name, parent, plptr);
  }
  else
  {
    std::shared_ptr<emr::Item> itemptr = env_model_repository_.getItemByName(name);
    if (container.last_modified > getContainer<Container>(itemptr).last_modified) 
    {
      // Update the item information
      std::shared_ptr<emr::ROSPayload<Container>> plptr = std::make_shared<emr::ROSPayload<Container>>(rospl);
      env_model_repository_.updateItem(name, plptr);
      // TEMOTO_INFO_STREAM("Updated item: " << name);
    }
  }
  return true;
}

std::vector<temoto_context_manager::ItemContainer> EMR_ROS_interface::EMRtoVector()
{
  std::vector<temoto_context_manager::ItemContainer> items;
  std::vector<std::shared_ptr<emr::Item>> root_items = env_model_repository_.getRootItems();
  for (const auto& item : root_items)
  {
    EMRtoVectorHelper(*item, items);
  }
  return items;
}

void EMR_ROS_interface::EMRtoVectorHelper(const emr::Item& currentItem, std::vector<temoto_context_manager::ItemContainer>& items)
{
  // Create empty container and fill it based on the payload
  temoto_context_manager::ItemContainer nc;
  
  nc.type = currentItem.getPayload()->getType();

  // Get the item payload as ROS msg
  // To do this we dynamically cast the base class to the appropriate
  //   derived class and call the getPayload() method
  if (nc.type == "OBJECT") 
  {
    temoto_context_manager::ObjectContainer rospl = 
      std::dynamic_pointer_cast<emr::ROSPayload<temoto_context_manager::ObjectContainer>>(currentItem.getPayload())
        ->getPayload();
    nc.serialized_container = temoto_core::serializeROSmsg(rospl);
    items.push_back(nc);
  }
  else if (nc.type == "MAP") 
  {
    temoto_context_manager::MapContainer rospl = 
      std::dynamic_pointer_cast<emr::ROSPayload<temoto_context_manager::MapContainer>>(currentItem.getPayload())
        ->getPayload();
    nc.serialized_container = temoto_core::serializeROSmsg(rospl);
    items.push_back(nc);
  }
  else if (nc.type == "COMPONENT") 
  {
    temoto_context_manager::ComponentContainer rospl = 
      std::dynamic_pointer_cast<emr::ROSPayload<temoto_context_manager::ComponentContainer>>(currentItem.getPayload())
        ->getPayload();
    nc.serialized_container = temoto_core::serializeROSmsg(rospl);
    items.push_back(nc);
  }
  else
  {
    // TEMOTO_ERROR_STREAM("Wrong type of container @ EMRtoVectorHelper: " << nc.type);
    return;
  }

  std::vector<std::shared_ptr<emr::Item>> children = currentItem.getChildren();
  for (uint32_t i = 0; i < children.size(); i++)
  {
    EMRtoVectorHelper(*children[i], items);
  }
}

/**
 * @brief Function to traverse and print out every item name in the tree
 * 
 * @param root 
 */
void EMR_ROS_interface::traverseEMR(const emr::Item& root)
{
  // TEMOTO_DEBUG_STREAM(root.getPayload()->getName());
  std::shared_ptr<emr::ROSPayload<temoto_context_manager::ObjectContainer>> msgptr = std::dynamic_pointer_cast<emr::ROSPayload<temoto_context_manager::ObjectContainer>>(root.getPayload());
  // TEMOTO_DEBUG_STREAM("tag_id: " << msgptr->getPayload().tag_id);
  std::vector<std::shared_ptr<emr::Item>> children = root.getChildren();
  for (uint32_t i = 0; i < children.size(); i++)
  {
    traverseEMR(*children[i]);
  }
}

} // namespace EMR_ROS_Interface