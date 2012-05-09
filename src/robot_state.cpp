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
 * Implementation of robot_state.h
 *
 * \author Bhaskara Marthi
 */

#include "robot_state.h"
#include <tf/transform_datatypes.h>

namespace ros_logging
{

using std::cerr;
using std::endl;
using std::cout;

using std::vector;
using std::string;

using mongo::BSONElement;
using mongo::BSONObj;

bool RobotState::isEmpty() const
{
  return !(joint_state.get());
}


gm::Pose::ConstPtr getPose (mongo::BSONObj b)
{
  BSONElement elt = b.getField("pose");
  cerr << "Processing " << elt.toString() << endl;
  if (elt.eoo())
    return gm::Pose::ConstPtr();
  else
  {
    vector<BSONElement> p = elt.Array();
    const double x = p[0].Double();
    const double y = p[1].Double();
    const double th = p[2].Double();
    gm::Pose::Ptr pose(new gm::Pose());
    pose->position.x = x;
    pose->position.y = y;
    pose->orientation = tf::createQuaternionMsgFromYaw(th);
    return pose;
  }
}

void RobotState::initializeJointState (const vector<string>& names)
{
  sm::JointState js;
  js.name = names;
  joint_state.reset(new sm::JointState(js));
  cerr << "Initialized joint state to " << *joint_state << endl;
}


// Update given a BSONObj representing diffs
void RobotState::update(BSONObj b)
{
  ROS_ASSERT(!isEmpty());
  if (!b.getField("pose_diff").eoo())
    pose = getPose(b);
  
  vector<BSONElement> indices = b.getField("indices").Array();
  vector<BSONElement> positions = b.getField("positions").Array();
  
  ROS_ASSERT(indices.size()==positions.size());
  for (size_t i=0; i<indices.size(); i++)
    joint_state->position[indices[i].Int()] = positions[i].Double();
}


} // namespace
