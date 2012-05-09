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

#include "time_warper.h"
#include "robot_state.h"
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <boost/optional.hpp>
#include <mongo/client/dbclient.h>
#include <boost/program_options.hpp>

namespace po=boost::program_options;

using std::cerr;
using std::endl;
using std::string;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::BSONElement;
using mongo::fromjson;
using boost::optional;

typedef boost::shared_ptr<mongo::DBClientConnection> ConnPtr;

namespace logging_tools
{


class Node
{
public:
  
  Node (const string& hostname, int port);

  /// Replay the robot states and poses between times \a t1 and \a t2
  /// Higher values of \a rate -> faster playback.  Times refer to
  /// wall time when the state was received, not ROS time (this only matters
  /// when the global use_sim_time ROS parameter is true).
  void replay (unsigned t1, unsigned t2, double rate);
  
private:

  /// Return the receipt time of the last key frame that is <= t
  double lastKeyframeBefore (unsigned t);
  
  /// Send out a tf message for a base pose
  void publishBasePose (const gm::Pose& p);
  
  /// Read the joint names upon startup
  vector<string> getNames ();

  ros::NodeHandle nh_;

  const string coll_;
  const double time_inc_;

  ConnPtr conn_;
  tf::TransformBroadcaster tfb_;

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

Node::Node(const string& hostname, int port) :
  coll_("ros_logging.robot_state_log"),
  time_inc_(0.1), 
  conn_(createConnection(hostname, port))
{}

double Node::lastKeyframeBefore(const unsigned t)
{
  return t;
}

void Node::publishBasePose (const gm::Pose& p)
{
  tf::StampedTransform trans;
  tf::poseMsgToTF(p, trans);
  trans.stamp_ = ros::Time::now();
  trans.frame_id_ = "map";
  trans.child_frame_id_ = "base_footprint";
  tfb_.sendTransform(trans);
}

vector<string> Node::getNames ()
{
  const string coll = "ros_logging.joint_state_names";
  BSONObj query = mongo::fromjson("{}");
  auto_ptr<mongo::DBClientCursor> cursor = conn_->query(coll, query);
  ROS_ASSERT_MSG(cursor->more(), "ros_logging.joint_state_names was empty");
  BSONObj item = cursor->next();
  BSONElement elt = item.getField("names");
  vector<BSONElement> names = elt.Array();
  vector<string> joint_names(names.size());
  for (size_t i=0; i<names.size(); i++)
    joint_names[i] = names[i].String();
  return joint_names;
}
  

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

  TimeWarper warper(ros::WallTime::now().toSec(), t1, t2, rate, time_inc_);
  RobotState state;
  state.initializeJointState(getNames());

  // Loop over items satisfying the query
  while (cursor->more() && ros::ok())
  {
    BSONObj item = cursor->next();
    const double t = item.getField("receipt_time").numberDouble();
    ROS_INFO("Considering item with stamp %.4f", t);
    state.update(item);
    if (t<t1)
      // We start at t_key<t1 because we need to start at a keyframe
      // Items with t<t1 are only used to update the state, and then we move on
      continue;
    
    optional<double> wait = warper.waitTime(t, ros::WallTime::now().toSec());
    if (wait)
    {
      ROS_INFO ("Waiting for %.4f seconds", *wait);
      ros::WallDuration(*wait).sleep();
      ROS_INFO ("Would now publish message with stamp %.4f", t);
      if (state.pose)
        publishBasePose(*state.pose);
    }
  }
}

} // namespace


time_t parseTime (const string& str)
{
  time_t current;
  time(&current);
  struct tm* t;
  t = localtime(&current);
  strptime(str.c_str(), "%H:%M:%S", t);
  return mktime(t);

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "replay_robot_state");

  po::options_description desc("Allowed options");
  string hostname;
  int port;
  time_t start_time, end_time;

  desc.add_options()
    ("help,h", "Display help message")
    ("start_time,s", po::value<time_t>(&start_time), "Start time")
    ("end_time,e", po::value<time_t>(&end_time), "End time")
    ("hostname,z", po::value<string>(&hostname)->default_value("localhost"),
     "Hostname for db server.  Defaults to localhost")
    ("port,p", po::value<int>(&port)->default_value(27017),
     "Port for db server.  Defaults to 27017");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << endl;
    return 1;
  }

  // Parse times
  /*
  time_t t1 = 1335820120; //parseTime(start_time);
  time_t t2 = 1335900000; //parseTime(end_time);
  */
  logging_tools::Node node(hostname, port);
  node.replay(start_time, end_time, 3.0);
  return 0;
}
