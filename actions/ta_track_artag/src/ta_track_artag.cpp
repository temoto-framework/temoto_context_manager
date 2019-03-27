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
#include <class_loader/class_loader.h>  // Class loader includes

#include "temoto_core/common/topic_container.h"
#include "temoto_context_manager/env_model_repository.h"

/* 
 * ACTION IMPLEMENTATION of TaTrackArtag 
 */
class TaTrackArtag : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaTrackArtag()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaTrackArtag constructed");
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

~TaTrackArtag()
{
  TEMOTO_INFO("TaTrackArtag destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

ros::NodeHandle nh_;
ros::Subscriber artag_subscriber_;
ros::Publisher tracked_object_publisher_;
uint32_t tag_id_;
//temoto_context_manager::ObjectPtr tracked_object_;
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  object_name = what_0_in.words_[0];

  temoto_nlp::Subject what_1_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_1_word_in = what_1_in.words_[0];
  std::string  what_1_data_0_in = boost::any_cast<std::string>(what_1_in.data_[0].value);
  temoto_core::TopicContainer topic_container = boost::any_cast<temoto_core::TopicContainer>(what_1_in.data_[1].value);
  emr::EnvironmentModelRepository* emr_ptr = 
    boost::any_cast<emr::EnvironmentModelRepository*>(what_1_in.data_[2].value);

  TEMOTO_INFO_STREAM("starting to track object: " << object_name);
  TEMOTO_INFO_STREAM("Receiving informantion on topics:");

  for (auto topic_pair : topic_container.getOutputTopics())
  {
    TEMOTO_INFO_STREAM("* type: " << topic_pair.first << "; topic: " << topic_pair.second);
  }
}

// void artagDataCb(ar_track_alvar_msgs::AlvarMarkers msg)
// {

//   // Look for the marker with the required tag id
//   for (auto& artag : msg.markers)
//   {
//     if (artag.id == tag_id_)
//     {
//       TEMOTO_INFO_STREAM( "AR tag with id = " << tag_id_ << " found");

//       // Update the pose of the object
//       tracked_object_->pose.pose = artag.pose.pose;
//       tracked_object_->pose.header = artag.header;

//       // Publish the tracked object
//       tracked_object_publisher_.publish(*tracked_object_);

//       // TODO: do something reasonable if multiple markers with the same tag id are present
//     }
//   }
// }

}; // TaTrackArtag class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTrackArtag, temoto_nlp::BaseTask);
