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
 * Replay the robot states saved by robot state logger
 *
 * \author Bhaskara Marthi
 */

#include "robot_state.h"
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <boost/optional.hpp>

using std::cerr;
using std::endl;
using std::string;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::fromjson;
using boost::optional;

typedef boost::shared_ptr<mongo::DBClientConnection> ConnPtr;

namespace ros_logging
{


class Node
{
public:
  
  Node ();

  /// Replay the robot states and poses between times \a t1 and \a t2
  /// Higher values of \a rate -> faster playback.  Times refer to
  /// wall time when the state was received, not ROS time (this only matters
  /// when the global use_sim_time ROS parameter is true).
  void replay (unsigned t1, unsigned t2, double rate);
  
private:

  /// Return the receipt time of the last key frame that is <= t
  double lastKeyframeBefore (unsigned t);

  ros::NodeHandle nh_;

  const string coll_;
  const double time_inc_;

  ConnPtr conn_;

};

// Internal function to connect to the mongo instance
// Can throw a Mongo exception if connection fails
ConnPtr createConnection (const string& host, const unsigned port)
{
  const string address = (boost::format("%1%:%2%") % host % port).str();
  ConnPtr conn(new mongo::DBClientConnection());
  conn->connect(address);
  return conn;
}

Node::Node() :
  coll_("ros_logging.robot_state_log"),
  time_inc_(0.1), 
  conn_(createConnection("localhost", 27017))
{}

double Node::lastKeyframeBefore(const unsigned t)
{
  return t;
}

// We're going to be playing back messages received between t1 and t2, at
// some rate; we're also going to be skipping messages to ensure that we
// publish at most one message every inc seconds.  This class wraps all of
// that logic and related state.
class TimeWarper
{
public:
  TimeWarper (double playback_start_time, double earliest_receipt_time,
              double latest_receipt_time, double rate, double inc) :
    playback_start_time_(playback_start_time),
    earliest_receipt_time_(earliest_receipt_time),
    latest_receipt_time_(latest_receipt_time), rate_(rate), inc_(inc),
    last_ind_(-1)
  {}

  // Given a new message with timestamp t, and given current time,
  // return how long, in seconds, to wait before publishing the message,
  // or an empty value if the message should be skipped, either because
  // it's in the same time window as an already published message, or because
  // it's too late to publish this one.
  optional<double> waitTime (double t)
  {
    const int ind = int(floor((t-earliest_receipt_time_)/(rate_*inc_)));
    if (ind>last_ind_)
    {
      last_ind_ = ind;
      const double publish_at = playback_start_time_+ind*inc_;
      if (publish_at > t)
        return publish_at-t;
    }
    return optional<double>();
  }

private:
  const double playback_start_time_;
  const double earliest_receipt_time_;
  const double latest_receipt_time_;
  const double rate_;
  const double inc_;
  int last_ind_;
};

void Node::replay (const unsigned t1, const unsigned t2, const double rate)
{
  const unsigned t_key = lastKeyframeBefore(t1);
  ROS_INFO_STREAM ("Starting playback from " << t1 << " to " << t2 << endl);
  
  // Generate the query
  BSONObj cond = BSON("$gte" << unsigned(t_key) << "$lte" << unsigned(t2));
  BSONObj q = BSON("receipt_time" << cond);
  BSONObjBuilder b;
  b.append("query", q);
  b.append("orderby", BSON("receipt_time" << 1));
  BSONObj query = b.obj();
  cerr << "Final query is: " << query.toString() << endl;
  auto_ptr<mongo::DBClientCursor> cursor = conn_->query(coll_, query);

  ros::WallTime now = ros::WallTime::now();
  TimeWarper warper(now.toSec(), t1, t2, rate, time_inc_);
  RobotState state;

  // Loop over items satisfying the query
  while (cursor->more() && ros::ok())
  {
    BSONObj item = cursor->next();
    const double t = item.getField("receipt_time").numberDouble();
    state.update(item);
    if (t<t1)
      // We start at t_key<t1 because we need to start at a keyframe
      // Items with t<t1 are only used to update the state, and then we move on
      continue;
    
    optional<double> wait = warper.waitTime(t);
    if (wait)
    {
      ros::WallDuration(*wait).sleep();
      ROS_INFO ("Would now publish message with stamp %.4f", t);
    }
  }
}

} // namespace


time_t parseTime (const char* str)
{
  time_t current;
  time(&current);
  struct tm* t;
  t = localtime(&current);
  strptime(str, "%H:%M:%S", t);
  return mktime(t);

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "replay_robot_state");
  if (argc!=3)
  {
    cerr << "Usage: " << argv[0] << " START_TIME END_TIME" << endl;
    return 1;
  }

  // Parse times
  time_t t1 = parseTime(argv[1]);
  time_t t2 = parseTime(argv[2]);
  ros_logging::Node node;
  node.replay(t1, t2, 3.0);
  return 0;
}
