#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_SERVICES_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/rmp/resource_manager_services.h"
#include "temoto_context_manager/GetNumber.h"
#include "temoto_context_manager/TrackObject.h"
#include "temoto_context_manager/UpdateEMR.h"
#include "temoto_context_manager/GetEMRNode.h"

namespace temoto_context_manager
{
  namespace srv_name
  {
    const std::string MANAGER = "temoto_context_manager";
    const std::string SYNC_OBJECTS_TOPIC = "/temoto_context_manager/"+MANAGER+"/sync_objects";
    const std::string SYNC_TRACKED_OBJECTS_TOPIC= "/temoto_context_manager/"+MANAGER+"/sync_tracked_objects";
    const std::string GET_NUMBER_SERVER = "get_number";
    const std::string TRACK_OBJECT_SERVER = "track_objects";

    const std::string MANAGER_2 = "temoto_context_manager_2";
    const std::string TRACKER_SERVER = "load_tracker";

    const std::string SERVER_UPDATE_EMR = "update_emr";
    const std::string SERVER_GET_EMR_NODE = "get_emr_node";
  }
}

static bool operator==(const temoto_context_manager::GetNumber::Request& r1,
                       const temoto_context_manager::GetNumber::Request& r2)
{
  return( r1.requested_int == r2.requested_int);
}

/**
 * @brief operator ==
 * @param r1
 * @param r2
 * @return
 */
static bool operator==(const temoto_context_manager::TrackObject::Request& r1,
                       const temoto_context_manager::TrackObject::Request& r2)
{
    return( r1.object_name == r2.object_name);
}

#endif
