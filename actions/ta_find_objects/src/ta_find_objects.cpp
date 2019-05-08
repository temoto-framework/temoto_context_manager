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
 * ACTION IMPLEMENTATION of TaFindObjects 
 */
class TaFindObjects : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaFindObjects()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaFindObjects constructed");
  artag_item_map_[0] = "coffee_mug";
  artag_item_map_[1] = "tv_remote";
  artag_item_map_[2] = "chair";
  artag_item_map_[3] = "scout_1";
  artag_item_map_[4] = "scout_2";
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

~TaFindObjects()
{
  TEMOTO_INFO("TaFindObjects destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

std::map<int, std::string> artag_item_map_;

ros::NodeHandle nh_;
ros::Subscriber artag_subscriber_;
ros::Publisher tracked_object_publisher_;
uint32_t tag_id_;
temoto_context_manager::ObjectContainer tracked_object_;
tf::TransformListener tf_listener;
std::string object_name;
temoto_context_manager::EnvModelInterface* emr_interface_ptr;
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  temoto_nlp::Subject what_1_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_1_word_in = what_1_in.words_[0];
  std::string  what_1_data_0_in = boost::any_cast<std::string>(what_1_in.data_[0].value);
  temoto_core::TopicContainer topic_container = boost::any_cast<temoto_core::TopicContainer>(what_1_in.data_[1].value);
  emr_interface_ptr = 
    boost::any_cast<temoto_context_manager::EnvModelInterface*>(what_1_in.data_[2].value);
  tag_id_ = tracked_object_.tag_id;
  
  TEMOTO_INFO_STREAM("Set up object finder for robot: " << temoto_core::common::getTemotoNamespace());
  std::string topic = topic_container.getOutputTopic("marker_data");
  TEMOTO_INFO_STREAM("Receiving information on topics:" << topic);
  artag_subscriber_ = nh_.subscribe(topic, 1000, &TaFindObjects::artagDataCb, this);

  for (auto topic_pair : topic_container.getOutputTopics())
  {
    TEMOTO_INFO_STREAM("* type: " << topic_pair.first << "; topic: " << topic_pair.second);
  }
}

void artagDataCb(ar_track_alvar_msgs::AlvarMarkers msg)
{

  // Look for markers we know the corresponding objects to
  for (auto& artag : msg.markers)
  {
    if (emr_interface_ptr->hasItem(artag_item_map_[artag.id])) 
    {
      object_name = artag_item_map_[artag.id];
      tracked_object_ = emr_interface_ptr->
            getObject(object_name);
      TEMOTO_INFO_STREAM("Updating pose of item " << object_name << " with ID " << artag.id);

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
    else
    {
      object_name = artag_item_map_[artag.id];

      if (object_name.empty()) continue;

      temoto_context_manager::ObjectContainer new_obj;
      new_obj.name = object_name;
      
      temoto_context_manager::MapContainer parent_map = 
            emr_interface_ptr->getNearestParentMap(
                temoto_core::common::getTemotoNamespace());
      new_obj.parent = parent_map.name;

      tracked_object_ = new_obj;

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
      temoto_context_manager::ItemContainer ic;
      ic.maintainer = temoto_core::common::getTemotoNamespace();
      ic.serialized_container = temoto_core::serializeROSmsg(tracked_object_);
      newPose.header = artag.pose.header;
      tracked_object_.pose = newPose;
      emr_interface_ptr->updateEmr(ic);
    }
  }
}

}; // TaFindObjects class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaFindObjects, temoto_nlp::BaseTask);
