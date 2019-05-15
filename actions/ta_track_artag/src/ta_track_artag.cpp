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
#include "temoto_context_manager/emr_ros_interface.h"
#include "tf/transform_listener.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/PoseStamped.h"

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
temoto_context_manager::ObjectContainer tracked_object_;
tf::TransformListener tf_listener;
std::string object_name;
std::shared_ptr<temoto_context_manager::EnvModelInterface> emr_interface_ptr;
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  object_name = what_0_in.words_[0];

  temoto_nlp::Subject what_1_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_1_word_in = what_1_in.words_[0];
  std::string  what_1_data_0_in = boost::any_cast<std::string>(what_1_in.data_[0].value);
  temoto_core::TopicContainer topic_container = boost::any_cast<temoto_core::TopicContainer>(what_1_in.data_[1].value);
  emr_interface_ptr = 
    boost::any_cast<std::shared_ptr<temoto_context_manager::EnvModelInterface>>(what_1_in.data_[2].value);
  tracked_object_ = emr_interface_ptr->getObject(object_name);
  tag_id_ = tracked_object_.tag_id;
  
  TEMOTO_INFO_STREAM("Starting to track object: " << object_name);
  std::string topic = topic_container.getOutputTopic("marker_data");
  TEMOTO_INFO_STREAM("Receiving information on topics:" << topic);
  artag_subscriber_ = nh_.subscribe(topic, 1000, &TaTrackArtag::artagDataCb, this);

  try
  {
  temoto_context_manager::MapContainer map = 
    emr_interface_ptr->
      getNearestParentMap(object_name);
  TEMOTO_WARN_STREAM("Highest parent map: " << map.name);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }


  for (auto topic_pair : topic_container.getOutputTopics())
  {
    TEMOTO_INFO_STREAM("* type: " << topic_pair.first << "; topic: " << topic_pair.second);
  }
}

void artagDataCb(ar_track_alvar_msgs::AlvarMarkers msg)
{

  // Look for the marker with the required tag id
  for (auto& artag : msg.markers)
  {
    if (artag.id == tag_id_)
    {
      TEMOTO_INFO_STREAM( "AR tag with id = " << tag_id_ << " found");

      // Update the pose of the object
      // tracked_object_->pose.pose = artag.pose.pose;
      artag.pose.header.frame_id = artag.header.frame_id;
      geometry_msgs::PoseStamped newPose;
      try
      {
        tf_listener.transformPose(tracked_object_.parent, artag.pose, newPose);
      }
      catch(tf2::InvalidArgumentException& e )
      {
        TEMOTO_ERROR(e.what());
      }
      
      newPose.header = artag.pose.header;
      emr_interface_ptr->updatePose(object_name, newPose);

      // Publish the tracked object
      // tracked_object_publisher_.publish(tracked_object_);

      // TODO: do something reasonable if multiple markers with the same tag id are present
    }
  }
}

}; // TaTrackArtag class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTrackArtag, temoto_nlp::BaseTask);
