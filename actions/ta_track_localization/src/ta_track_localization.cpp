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
#include "geometry_msgs/PoseWithCovarianceStamped.h"

/* 
 * ACTION IMPLEMENTATION of TaTrackLocalization 
 */
class TaTrackLocalization : public temoto_nlp::BaseTask
{
public:

/* REQUIRED BY TEMOTO */
TaTrackLocalization()
{
  // ---> YOUR CONSTRUCTION ROUTINES HERE <--- //
  TEMOTO_INFO("TaTrackLocalization constructed");
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

~TaTrackLocalization()
{
  TEMOTO_INFO("TaTrackLocalization destructed");
}

/********************* END OF REQUIRED PUBLIC INTERFACE *********************/


private:

ros::NodeHandle nh_;
ros::Subscriber amcl_subscriber;
ros::Publisher tracked_object_publisher_;
uint32_t tag_id_;
temoto_context_manager::RobotContainer tracked_robot_;
tf::TransformListener tf_listener;
std::string robot_name;
std::shared_ptr<temoto_context_manager::EnvModelInterface> emr_interface_ptr;
    
/*
 * Interface 0 body
 */
void startInterface_0()
{
  /* EXTRACTION OF INPUT SUBJECTS */
  temoto_nlp::Subject what_0_in = temoto_nlp::getSubjectByType("what", input_subjects);
  robot_name = what_0_in.words_[0];

  temoto_nlp::Subject what_1_in = temoto_nlp::getSubjectByType("what", input_subjects);
  std::string  what_1_word_in = what_1_in.words_[0];
  std::string  what_1_data_0_in = boost::any_cast<std::string>(what_1_in.data_[0].value);
  temoto_core::TopicContainer topic_container = boost::any_cast<temoto_core::TopicContainer>(what_1_in.data_[1].value);
  emr_interface_ptr = 
    boost::any_cast<std::shared_ptr<temoto_context_manager::EnvModelInterface>>(what_1_in.data_[2].value);
  tracked_robot_ = emr_interface_ptr->getRobot(robot_name);
  
  TEMOTO_INFO_STREAM("Starting to track robot: " << robot_name);
  std::string topic = topic_container.getOutputTopic("pose_out");
  TEMOTO_INFO_STREAM("Receiving information on topic:" << topic);
  amcl_subscriber = nh_.subscribe(topic, 1000, &TaTrackLocalization::amclDataCb, this);

  for (auto topic_pair : topic_container.getOutputTopics())
  {
    TEMOTO_INFO_STREAM("* type: " << topic_pair.first << "; topic: " << topic_pair.second);
  }
}

void amclDataCb(geometry_msgs::PoseWithCovarianceStamped msg)
{

  // Look for the marker with the required tag id
  TEMOTO_DEBUG_STREAM("Updating robot " << robot_name << " pose in ta_track_localization.");
  // Update the pose of the object
  // tracked_robot_->pose.pose = artag.pose.pose;
  // msg.pose.header.frame_id = msg.header.frame_id;
  geometry_msgs::PoseStamped pose_in;
  pose_in.header = msg.header;
  pose_in.pose.position.x = msg.pose.pose.position.x;
  pose_in.pose.position.y = msg.pose.pose.position.y;
  pose_in.pose.position.z = msg.pose.pose.position.z;
  pose_in.pose.orientation.x = msg.pose.pose.orientation.x;
  pose_in.pose.orientation.y = msg.pose.pose.orientation.y;
  pose_in.pose.orientation.z = msg.pose.pose.orientation.z;
  pose_in.pose.orientation.w = msg.pose.pose.orientation.w;

  geometry_msgs::PoseStamped newPose;
  try
  {
    tf_listener.transformPose(tracked_robot_.parent, pose_in, newPose);
  }
  catch(tf2::InvalidArgumentException& e )
  {
    TEMOTO_ERROR(e.what());
  }
  
  newPose.header = msg.header;
  emr_interface_ptr->updatePose(tracked_robot_.name, newPose);
  // tracked_robot_.pose = newPose;
  // temoto_context_manager::ItemContainer item;
  // item.type = emr_ros_interface::emr_containers::ROBOT;
  // item.maintainer = temoto_core::common::getTemotoNamespace();
  // item.serialized_container = temoto_core::serializeROSmsg(tracked_robot_);
  // emr_interface_ptr->updateEmr(item, false);

  // Publish the tracked object
  // tracked_object_publisher_.publish(tracked_robot_);

}

}; // TaTrackLocalization class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTrackLocalization, temoto_nlp::BaseTask);
