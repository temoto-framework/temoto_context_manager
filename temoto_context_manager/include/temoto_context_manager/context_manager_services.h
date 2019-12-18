/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Author: Robert Valner */

#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_SERVICES_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_SERVICES_H

#include <string>
#include "temoto_core/trr/resource_registrar_services.h"
#include "temoto_context_manager/TrackObject.h"
#include "temoto_context_manager/UpdateEmr.h"
#include "temoto_context_manager/GetEMRItem.h"
#include "temoto_context_manager/GetEMRVector.h"

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

    const std::string SERVER_UPDATE_EMR = MANAGER + "/update_emr";
    const std::string SERVER_GET_EMR_ITEM = "get_emr_item";
    const std::string SERVER_GET_EMR_VECTOR = "get_emr_vector";
  }
}

static bool operator==(const temoto_context_manager::TrackObject::Request& r1,
                       const temoto_context_manager::TrackObject::Request& r2)
{
    return( r1.object_name == r2.object_name);
}

#endif
