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

#ifndef TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_SERVICES_H
#define TEMOTO_CONTEXT_MANAGER__CONTEXT_MANAGER_SERVICES_H

#include <string>
#include "temoto_context_manager/UpdateEmr.h"
#include "temoto_context_manager/GetEMRItem.h"
#include "temoto_context_manager/GetEMRVector.h"

namespace temoto_context_manager
{
  namespace srv_name
  {
    const std::string MANAGER = "temoto_context_manager";
    const std::string SYNC_OBJECTS_TOPIC = "/temoto_context_manager/"+MANAGER+"/sync_objects";

    const std::string SERVER_UPDATE_EMR = MANAGER + "/update_emr";
    const std::string SERVER_GET_EMR_ITEM = "get_emr_item";
    const std::string SERVER_GET_EMR_VECTOR = "get_emr_vector";
  }
}

#endif
