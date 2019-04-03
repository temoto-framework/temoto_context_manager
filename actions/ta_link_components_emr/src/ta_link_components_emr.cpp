
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
 *    https://utnuclearroboticspublic.github.io/temoto2
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include "temoto_nlp/base_task/base_task.h"    // The base task
#include <class_loader/class_loader.h>  // Class loader include

#include "temoto_component_manager/component_manager_services.h"
#include "temoto_context_manager/emr_ros_interface.h"
#include "temoto_context_manager/emr_item_to_component_link.h"

/* 
 * ACTION IMPLEMENTATION of TaLinkComponentsEmr 
 */
class TaLinkComponentsEmr : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaLinkComponentsEmr()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaLinkComponentsEmr constructed");
}
    
/* REQUIRED BY TEMOTO */
void startTask(temoto_nlp::TaskInterface task_interface)
{
  input_subjects = task_interface.input_subjects_;
  switch (task_interface.id_)
  {
        
    // Interface 0
    case 0:
      startInterface_0();
      break;

  }
}

/* REQUIRED BY TEMOTO */
std::vector<temoto_nlp::Subject> getSolution()
{
  return output_subjects;
}

~TaLinkComponentsEmr()
{
  TEMOTO_INFO("TaLinkComponentsEmr destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

ros::NodeHandle nh_;
ros::ServiceClient list_components_client_;
ros::ServiceClient list_pipes_client_;
std::vector<temoto_component_manager::Component> component_info_msgs_;
std::vector<temoto_component_manager::Pipe> pipe_info_msgs_;
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_0_word_in = what_0_in.words_[0];
  emr_ros_interface::EmrRosInterface* eri = 
    boost::any_cast<emr_ros_interface::EmrRosInterface*>(what_0_in.data_[0].value);

  temoto_nlp::Subject what_1_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_1_word_in = what_1_in.words_[0];
  temoto_context_manager::ComponentToEmrRegistry* c_emr_reg = 
    boost::any_cast<temoto_context_manager::ComponentToEmrRegistry*>(what_1_in.data_[0].value);

  // Initialize the list components service client
  list_components_client_ = nh_.serviceClient<temoto_component_manager::ListComponents>(
    temoto_component_manager::srv_name::LIST_COMPONENTS_SERVER);

  // Initialize the list pipes service client
  list_pipes_client_ = nh_.serviceClient<temoto_component_manager::ListPipes>(
    temoto_component_manager::srv_name::LIST_PIPES_SERVER);

  /*
   * Loop until this action is required to stop
   */ 
  while(!stop_task_)
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
        if (eri->hasItem(ci.component_name) && !c_emr_reg->hasLink(ci.component_name))
        {
          c_emr_reg->addLink(ci, ci.component_name);
        }
        else if (!eri->hasItem(ci.component_name) && c_emr_reg->hasLink(ci.component_name))
        {
          c_emr_reg->removeLink(ci.component_name);
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
      pipe_info_msgs_ = list_pipes_srvmsg.response.pipe_infos;
      for(const auto& pi : pipe_info_msgs_)
      {
        /*
         * TODO: Add the pipes
         */ 
      }
    }
    else
    {
      TEMOTO_WARN_STREAM("Failed to call the Component Manager service: " 
        << temoto_component_manager::srv_name::LIST_PIPES_SERVER);
    }
    
    ros::Duration(10).sleep();
  }
}

}; // TaLinkComponentsEmr class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaLinkComponentsEmr, temoto_nlp::BaseTask);
