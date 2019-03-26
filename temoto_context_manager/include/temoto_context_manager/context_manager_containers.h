#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_CONTAINERS_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_CONTAINERS_H

#include "temoto_context_manager/ObjectContainer.h"
#include "temoto_context_manager/MapContainer.h"
#include "temoto_context_manager/ItemContainer.h"
#include "temoto_context_manager/ComponentContainer.h"
#include "temoto_core/common/topic_container.h"
#include "temoto_context_manager/env_model_repository.h"
namespace temoto_context_manager
{

// Define the type of the data that is going to be synchronized
typedef std::vector<ObjectContainer> Objects;
typedef std::shared_ptr<ObjectContainer> ObjectPtr;
typedef std::vector<ObjectPtr> ObjectPtrs;
typedef std::vector<ItemContainer> Items;
typedef std::shared_ptr<emr::Item> ItemPtr;
// ObjectContainer comparison operator
bool operator==(const ObjectContainer& ob1, const ObjectContainer& ob2);

} // temoto_context_manager namespace

#endif
