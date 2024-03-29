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
 * Ros node that logs rosout messages to db
 *
 * \author Bhaskara Marthi
 */

#include <logging_tools/mongo_logger.h>
#include <ros/ros.h>

namespace logging_tools
{

using rosgraph_msgs::Log;

class Node
{
public:

  Node ();
  
  void logCB (const Log& l);

private:
  
  ros::NodeHandle nh_;
  
  MongoLogger logger_;
  ros::Subscriber sub_;
};


// Constructor sets up the db connection and subscription
Node::Node () :
  logger_("ros_logging"), sub_(nh_.subscribe("rosout_agg", 1000,
                                             &Node::logCB, this))
{}

void Node::logCB (const Log& l)
{
  logger_.write(l, ros::WallTime::now());
}

} // namespace

using std::cerr;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "rosout_logger_node");
  ros::NodeHandle nh; // Otherwise the ros::ok will fail on iteration 2
  while (ros::ok())
  {
    try
    {
      logging_tools::Node node;
      cerr << "Connected to db instance\n";
      ros::spin();
      break;
    }
    catch (mongo::ConnectException& e)
    {
      cerr << "Waiting for db connection\n";
      ros::WallDuration(1.0).sleep();
    }
  }
  return 0;
}
