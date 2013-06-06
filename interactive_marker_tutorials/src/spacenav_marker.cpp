/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::string spacenav_IM_name = "spacenav_marker";
ros::Subscriber spacenav_joy_sub;
// %EndTag(vars)%

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.6;
  marker.scale.y = msg.scale * 0.6;
  marker.scale.z = msg.scale * 0.6;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

void spaceNavJoyCallback(const sensor_msgs::Joy& spacenav_joy){
  InteractiveMarker int_marker;
  server->get(spacenav_IM_name, int_marker);

  geometry_msgs::Pose pose = int_marker.pose;
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);

  float rotate_threshould = 0.3;
  float translate_threshould = 0.1;
  float rotate_factor = 0.02;
  float translate_factor = 0.01;

  if(( spacenav_joy.axes[3] < -rotate_threshould ) || ( rotate_threshould < spacenav_joy.axes[3] )){  
    tf_pose = tf_pose*tf::Transform(tf::Quaternion(tf::Vector3(1, 0, 0), spacenav_joy.axes[3] * rotate_factor),
				    tf::Vector3(0, 0, 0));
  }
  if(( spacenav_joy.axes[4] < -rotate_threshould ) || ( rotate_threshould < spacenav_joy.axes[4] )){
    tf_pose = tf_pose*tf::Transform(tf::Quaternion(tf::Vector3(0, 1, 0), spacenav_joy.axes[4] * rotate_factor),
				    tf::Vector3(0, 0, 0));
  }
  if(( spacenav_joy.axes[5] < -rotate_threshould ) || ( rotate_threshould < spacenav_joy.axes[5] )){
    tf_pose = tf_pose*tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), spacenav_joy.axes[5] * rotate_factor),
				    tf::Vector3(0, 0, 0));
  }
  if(( spacenav_joy.axes[0] < -translate_threshould ) || ( translate_threshould < spacenav_joy.axes[0] )){
    tf_pose = tf_pose*tf::Transform(tf::Quaternion(tf::Vector3(1, 0, 0), 0),
				    tf::Vector3(spacenav_joy.axes[0]*translate_factor, 0, 0));
  }
  if(( spacenav_joy.axes[1] < -translate_threshould ) || ( translate_threshould < spacenav_joy.axes[1] )){
    tf_pose = tf_pose*tf::Transform(tf::Quaternion(tf::Vector3(0, 1, 1), 0),
				    tf::Vector3(0, spacenav_joy.axes[1]*translate_factor, 0));
  }
  if(( spacenav_joy.axes[2] < -translate_threshould ) || ( translate_threshould < spacenav_joy.axes[2] )){
    tf_pose = tf_pose*tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 0),
				    tf::Vector3(0, 0, spacenav_joy.axes[2]*translate_factor));
  }
  tf::poseTFToMsg(tf_pose, pose);
  int_marker.pose=pose;
  server->insert(int_marker);
  server->applyChanges();
}

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  InteractiveMarker int_marker;
  InteractiveMarkerControl control;
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

////////////////////////////////////////////////////////////////////////////////////

// %Tag(childMarker)%
void makeChildMarker(std::string name, std::string parent)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = parent;
  int_marker.scale = 1.0;

  int_marker.name = name;
  int_marker.description = name;

  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;

  int_marker.pose = pose;

  // insert a box
  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  server->applyChanges();
}
// %EndTag(childMarker)%

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::MultiThreadedSpinner spinner(4);
  ros::init(argc, argv, "spacenav_controls");
  ros::NodeHandle n;

  spacenav_joy_sub = n.subscribe("/spacenav/joy", 1000, spaceNavJoyCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("spacenav_controls","",false) );

  ros::Duration(0.1).sleep();

  makeChildMarker("spacenav_marker", "base_link");
  spinner.spin();

  server.reset();
}
// %EndTag(main)%
