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

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <mongo/client/dbclient.h>

using std::cerr;
using std::endl;
using std::string;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::fromjson;

typedef boost::shared_ptr<mongo::DBClientConnection> ConnPtr;

namespace ros_logging
{

class Node
{
public:
  
  Node ();

  void replay (time_t t1, time_t t2, double rate);
  
private:

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

void Node::replay (const time_t t1, const time_t t2, const double rate)
{
  ROS_INFO_STREAM ("Starting playback from " << t1 << " to " << t2 << endl);

  // Generate the query
  BSONObj cond = BSON("$gte" << unsigned(t1) << "$lte" << unsigned(t2));
  BSONObj q = BSON("receipt_time" << cond);
  BSONObjBuilder b;
  b.append("query", q);
  b.append("orderby", BSON("receipt_time" << 1));
  BSONObj query = b.obj();
  cerr << "Final query is: " << query.toString() << endl;
  
  auto_ptr<mongo::DBClientCursor> cursor = conn_->query(coll_, query);

  ros::WallTime t0 = ros::WallTime::now();
  int last_ind = -1;
  while (cursor->more() && ros::ok()
  {
    BSONObj item = cursor->next();
    const double t = item.getField("receipt_time").numberDouble();
    int ind = int(floor((t-t1)/(rate*time_inc_)));
    if (ind>last_ind)
    {
      last_ind = ind;
      ros::WallTime publish_at(t0+ros::WallDuration(ind*time_inc_));
      ros::WallDuration w = publish_at-ros::WallTime::now();
      if (w.toSec()>0)
        w.sleep();
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