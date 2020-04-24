
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://temoto-telerobotics.github.io
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include <class_loader/class_loader.hpp>
#include "ta_emr_component_linker/temoto_action.h"
#include "ta_emr_component_linker/macros.h"

#include "temoto_component_manager/component_manager_services.h"
#include "temoto_context_manager/emr_ros_interface.h"
#include "temoto_context_manager/emr_item_to_component_link.h"

/* 
 * ACTION IMPLEMENTATION of TaEmrComponentLinker 
 */
class TaEmrComponentLinker : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaEmrComponentLinker()
{
  std::cout << __func__ << " constructed\n";
}

// REQUIRED BY TEMOTO
void executeTemotoAction()
{
  // Input parameters
  temoto_context_manager::ComponentToEmrRegistry* c_em_reg =
    GET_PARAMETER("emr-to-component registry", temoto_context_manager::ComponentToEmrRegistry*);
  std::shared_ptr<temoto_context_manager::EnvModelInterface> emr_i = 
    GET_PARAMETER("emr", std::shared_ptr<temoto_context_manager::EnvModelInterface>);

  // Initialize the list components service client
  list_components_client_ = nh_.serviceClient<temoto_component_manager::ListComponents>(
    temoto_component_manager::srv_name::LIST_COMPONENTS_SERVER);

  // Initialize the list pipes service client
  list_pipes_client_ = nh_.serviceClient<temoto_component_manager::ListPipes>(
    temoto_component_manager::srv_name::LIST_PIPES_SERVER);

  /*
   * Loop until this action is required to stop
   */ 
  while(actionOk())
  {
    /*
     * Get the components
     */ 
    temoto_component_manager::ListComponents list_components_srvmsg;
    list_components_srvmsg.request.type = ""; // List all components

    if (list_components_client_.call(list_components_srvmsg))
    {
      /*
       * Start finding liks between EMR items and Component Manager components.
       * Link is created if EMR item name is equal to component name
       */
      component_info_msgs_ = list_components_srvmsg.response.component_infos;
      for(const auto& ci : component_info_msgs_)
      {
        if (emr_i->hasItem(ci.component_name) && !c_em_reg->hasLink(ci.component_name))
        {
          TEMOTO_INFO_STREAM("Linking component: " << ci.component_name);
          c_em_reg->addLink(ci, ci.component_name);
        }
        else if (!emr_i->hasItem(ci.component_name) && c_em_reg->hasLink(ci.component_name))
        {
          TEMOTO_INFO_STREAM("Un-linking component: " << ci.component_name);
          c_em_reg->removeLink(ci.component_name);
        }
      }
    }
    else
    {
      TEMOTO_WARN_STREAM("Failed to call the Component Manager service: " 
        << temoto_component_manager::srv_name::LIST_COMPONENTS_SERVER);
    }

    /*
     * Get the pipes
     */
    temoto_component_manager::ListPipes list_pipes_srvmsg;
    list_pipes_srvmsg.request.type = ""; // List all components
  
    if (list_pipes_client_.call(list_pipes_srvmsg))
    {
      temoto_context_manager::CategorizedPipes cat_pipes;
      pipe_info_msgs_ = list_pipes_srvmsg.response.pipe_infos;
      for(const auto& pi : pipe_info_msgs_)
      {
        TEMOTO_DEBUG_STREAM("Got pipe: " << pi.pipe_type);
        cat_pipes[pi.pipe_type].push_back(pi);
      }
      c_em_reg->setPipes(cat_pipes);
    }
    else
    {
      TEMOTO_WARN_STREAM("Failed to call the Component Manager service: " 
        << temoto_component_manager::srv_name::LIST_PIPES_SERVER);
    }
    ros::Duration(10).sleep();
  }
}

// Destructor
~TaEmrComponentLinker()
{
  TEMOTO_INFO("Action instance destructed");
}

private:
  ros::NodeHandle nh_;
  ros::ServiceClient list_components_client_;
  ros::ServiceClient list_pipes_client_;
  std::vector<temoto_component_manager::Component> component_info_msgs_;
  std::vector<temoto_component_manager::Pipe> pipe_info_msgs_;

}; // TaEmrComponentLinker class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaEmrComponentLinker, ActionBase);
