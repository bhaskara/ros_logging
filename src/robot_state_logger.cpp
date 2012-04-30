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
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <mongo/client/dbclient.h>

namespace ros_logging
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;

using std::vector;
using std::cerr;
using std::endl;
using std::string;
using std::pair;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::BSONArrayBuilder;

typedef boost::mutex::scoped_lock Lock;
typedef boost::shared_ptr<mongo::DBClientConnection> ConnPtr;


// Represents the joint angles and base position
struct RobotState
{
  RobotState (sm::JointState::ConstPtr js, gm::Pose::ConstPtr pose) :
    joint_state(js), pose(pose)
  {}

  RobotState ()
  {}

  bool isEmpty() const
  {
    return !(joint_state.get());
  }
  
  sm::JointState::ConstPtr joint_state;
  gm::Pose::ConstPtr pose;
};

typedef pair<size_t, float> Diff;

// Represents diffs between successive states
struct Diffs
{
  vector<Diff> new_joint_states;
  
  // new_pose is only considered if pose_diff is true
  // It can be empty if the new pose is unknown
  bool pose_diff;
  bool is_keyframe;
  gm::Pose::ConstPtr new_pose;

  Diffs () : pose_diff(false), is_keyframe(false) {}
};


class Node
{
public:

  Node ();
  
  void jointStateCB (sm::JointState::ConstPtr js);

private:

  Diffs getDiffs (const RobotState& last, const RobotState& current);

  // Return current base pose, or null if pose not available
  gm::Pose::ConstPtr getBasePose(const ros::Time& t);

  // Save a set of diffs to the database
  void saveToDB (const Diffs& diffs);
  
  void updateIgnoredJoints (const sm::JointState& s);
  
  ros::NodeHandle nh_;
  boost::mutex mutex_;

  const ros::Duration processing_interval_;
  const ros::Duration keyframe_interval_;
  const std::string base_frame_;
  const std::string fixed_frame_;
  vector<bool> joint_ignored_;
  const string diff_coll_;
  
  // Distance threshold for considering two joint angles to be the same
  // Note that this isn't quite right due to the existence of translational
  // joints (torso and gripper), but we'll just use the same number for meters
  // and radians for now
  const double distance_threshold_;

  ros::Time last_processed_;
  ros::Time last_keyframe_;
  RobotState last_saved_state_;

  tf::TransformListener tf_;
  ConnPtr conn_;
  ros::Subscriber state_sub_;

};


void Node::saveToDB (const Diffs& diffs)
{
  bool need_to_save = false;
  BSONObjBuilder builder;

  // Add joint state diffs
  BSONArrayBuilder ind_builder, pos_builder;
  BOOST_FOREACH (const Diff& d, diffs.new_joint_states) 
  {
    ind_builder.append(int(d.first));
    pos_builder.append(d.second);
    need_to_save = true;
  }
  builder.append("indices", ind_builder.arr());
  builder.append("positions", pos_builder.arr());

  // Pose diff if necessary
  if (diffs.pose_diff)
  {
    need_to_save = true;
    builder.append("pose_diff", true);
    if (diffs.new_pose.get())
    {
      // Store the pose as an array [x,y,theta] so we can create a 
      // 2d geospatial index over it
      BSONArrayBuilder pose_builder;
      pose_builder.append(diffs.new_pose->position.x);
      pose_builder.append(diffs.new_pose->position.y);
      pose_builder.append(tf::getYaw(diffs.new_pose->orientation));
      builder.append("pose", pose_builder.arr());
    }
  }

  if (need_to_save)
  {
    builder.append("receipt_time", ros::WallTime::now().toSec());
    if (diffs.is_keyframe)
      builder.append("keyframe", true);
    BSONObj item = builder.obj();
    cerr << "Inserting " << item.toString().c_str() << endl;
    conn_->insert(diff_coll_, item);
  }
}

// Internal function to connect to the mongo instance
// Can throw a Mongo exception if connection fails
ConnPtr createConnection (const string& host, const unsigned port)
{
  const string address = (boost::format("%1%:%2%") % host % port).str();
  ConnPtr conn(new mongo::DBClientConnection());
  conn->connect(address);
  return conn;
}

Node::Node () :
  processing_interval_(0.1), keyframe_interval_(60.0),
  base_frame_("base_footprint"), fixed_frame_("map"),
  diff_coll_("ros_logging.robot_state_log"),
  distance_threshold_(0.01), conn_(createConnection("localhost", 27017)),
  state_sub_(nh_.subscribe("joint_states", 1, &Node::jointStateCB, this))
{
  conn_->ensureIndex(diff_coll_, BSON("receipt_time" << 1));
  conn_->ensureIndex(diff_coll_, BSON("keyframe" << 1));
  conn_->ensureIndex(diff_coll_, BSON("pose" << "2d"));
}

gm::Pose::ConstPtr Node::getBasePose (const ros::Time& t)
{
  bool found = tf_.waitForTransform(fixed_frame_, base_frame_, t,
                                    processing_interval_);
  if (found)
  {
    try
    {
      tf::StampedTransform trans;
      tf_.lookupTransform(fixed_frame_, base_frame_, t, trans);
      gm::Pose::Ptr p(new gm::Pose());
      poseTFToMsg(trans, *p);
      return p;
    }
    catch (tf::TransformException& e)
    {
      ROS_INFO_THROTTLE(1.0, "Skipping due to lack of transform from %s to %s",
                        base_frame_.c_str(), fixed_frame_.c_str());
    }
  }
  return gm::Pose::ConstPtr();
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
  // The first time round, figure out which joints to ignore
  if (joint_ignored_.size()==0)
    updateIgnoredJoints(*m);
  
  // Only save every so often
  if (ros::Time::now() <= last_processed_+processing_interval_)
    return;

  RobotState rs(m, getBasePose(m->header.stamp));
  
  Diffs diffs = getDiffs(last_saved_state_, rs);
  saveToDB(diffs);
  last_saved_state_ = rs;
}

float poseDistance (const gm::Pose& p1, const gm::Pose& p2)
{
  const float dx = p1.position.x-p2.position.x;
  const float dy = p1.position.y-p2.position.y;
  const float dtheta = tf::getYaw(p1.orientation)-tf::getYaw(p2.orientation);
  return sqrt(dx*dx+dy*dy+dtheta*dtheta);
}

Diffs Node::getDiffs (const RobotState& last, const RobotState& current)
{
  Diffs diffs;
  const ros::Time now = ros::Time::now();;
  
  // Use the fact that JointState is empty iff this is the first time around
  if (last.isEmpty() || now >= last_keyframe_+keyframe_interval_)
  {
    last_keyframe_ = now;
    if (last.isEmpty())
      cerr << "Last is empty, so using all diffs" << endl;
    else
      cerr << "Keyframe interval elapsed, so saving entire joint state" << endl;
    diffs.is_keyframe = true;
    for (size_t i=0; i<current.joint_state->position.size(); i++)
      diffs.new_joint_states.push_back(Diff(i, current.joint_state->position[i]));
    diffs.pose_diff = true;
    diffs.new_pose = current.pose;
  }
  else
  {
    for (size_t i=0; i<current.joint_state->position.size(); i++)
    {
      if (fabs(current.joint_state->position[i]-
               last.joint_state->position[i])>distance_threshold_)
      {
        cerr << "Joint " << current.joint_state->name[i] << " value " <<
          current.joint_state->position[i] << " differs from " <<
          last.joint_state->position[i] << endl;
        diffs.new_joint_states.push_back(Diff(i, current.joint_state->position[i]));
      }
    }

    if ((current.pose.get() && !last.pose.get()) ||
        (!current.pose.get() && last.pose.get()) ||
        ((current.pose.get() && last.pose.get() &&
          poseDistance(*current.pose, *last.pose)>distance_threshold_)))
    {
      cerr << "Current pose: " << (bool)current.pose.get() << "; last pose: "
           << (bool)last.pose.get() << endl;
      diffs.pose_diff = true;
      diffs.new_pose = current.pose;
    }
  }
  return diffs;
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "robot_state_logger");
  ros_logging::Node node;
  ros::spin();
  return 0;
}
