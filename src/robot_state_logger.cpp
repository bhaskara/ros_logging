/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *
 */

/**
 * \file 
 * 
 * Ros node that logs robot joint states and possibly position of robot in world
 *
 * \author Bhaskara Marthi
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

namespace ros_logging
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;

using boost::optional;
using std::vector;
using std::string;

typedef boost::mutex::scoped_lock Lock;

class Node
{
public:

  Node ();
  
  void jointStateCB (sm::JointState::ConstPtr js);

private:

  bool sufficientlyClose(const sm::JointState& s1,
                         const sm::JointState& s2);

  optional<gm::Pose> getBasePose(const ros::Time& t);

  void saveToDB (const sm::JointState& s, const optional<gm::Pose>& p);
  
  void updateIgnoredJoints (const sm::JointState& s);
  
  ros::NodeHandle nh_;
  boost::mutex mutex_;

  const ros::Duration processing_interval_;
  const std::string base_frame_;
  const std::string fixed_frame_;
  vector<bool> joint_ignored_;
  
  // Distance threshold for considering two joint angles to be the same
  // Note that this isn't quite right due to the existence of translational
  // joints (torso and gripper), but we'll just use the same number for meters
  // and radians for now
  const double angle_distance_threshold_;

  ros::Time last_processed_;
  sm::JointState::ConstPtr last_saved_state_;

  tf::TransformListener tf_;
  ros::Subscriber state_sub_;

};


bool Node::sufficientlyClose (const sm::JointState& s1,
                              const sm::JointState& s2)
{
  ROS_ASSERT_MSG(s1.position.size()==s2.position.size(),
             "Joint states had different lengths %zu and %zu",
             s1.position.size(), s2.position.size());
  
  for (size_t i=0; i<s1.position.size(); i++)
  {
    if (fabs(s1.position[i]-s2.position[i])>angle_distance_threshold_
        && !joint_ignored_[i])
    {
      ROS_INFO ("Joint %s changed from %.4f to %.4f",
                s1.name[i].c_str(), s1.position[i], s2.position[i]);
      return false;
    }
  }
  return true;
}

void Node::saveToDB (const sm::JointState& s,
                     const optional<gm::Pose>& p)
{
  ROS_INFO ("Saving pose to db");
}

Node::Node () :
  processing_interval_(0.1), base_frame_("base_footprint"),
  fixed_frame_("map"), angle_distance_threshold_(0.01),
  state_sub_(nh_.subscribe("joint_states", 1, &Node::jointStateCB, this))
{
  
}

optional<gm::Pose> Node::getBasePose (const ros::Time& t)
{
  bool found = tf_.waitForTransform(fixed_frame_, base_frame_, t,
                                    processing_interval_);
  if (found)
  {
    try
    {
      tf::StampedTransform trans;
      tf_.lookupTransform(fixed_frame_, base_frame_, t, trans);
      gm::Pose p;
      poseTFToMsg(trans, p);
      return p;
    }
    catch (tf::TransformException& e)
    {
      ROS_INFO_THROTTLE(1.0, "Skipping due to lack of transform from %s to %s",
                        base_frame_.c_str(), fixed_frame_.c_str());
    }
  }
  return optional<gm::Pose>();
}

void Node::updateIgnoredJoints (const sm::JointState& m)
{
  joint_ignored_.resize(m.name.size());
  for (size_t i=0; i<m.name.size(); i++)
  {
    if (m.name[i].find("laser_tilt")!=string::npos ||
        m.name[i].find("caster")!=string::npos)
      joint_ignored_.at(i) = true;
    else
      joint_ignored_.at(i) = false;
  }
}

void Node::jointStateCB (sm::JointState::ConstPtr m)
{
  if (joint_ignored_.size()==0)
    updateIgnoredJoints(*m);
  if (ros::Time::now() <= last_processed_+processing_interval_)
    return;

  if (last_saved_state_&&sufficientlyClose(*last_saved_state_, *m))
    return;

  last_saved_state_ = m;
  saveToDB(*m, getBasePose(m->header.stamp));
}


} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "robot_state_logger");
  ros_logging::Node node;
  ros::spin();
  return 0;
}
