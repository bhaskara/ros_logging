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
 * Search the log for matching messages
 *
 * \author Bhaskara Marthi
 */

#include <ros_logging/mongo_logger.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/filesystem.hpp>
#include <ctime>
#include <iostream>
#include <fstream>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using ros_logging::LogItem;
using ros_logging::MongoLogger;
using boost::algorithm::trim_copy;
namespace po=boost::program_options;

int writeSessionStart (const double t)
{
  string session_path("/tmp/ros_logging_session_start");
  std::ofstream f;
  f.open(session_path.c_str());
  if (!f.is_open())
  {
    cerr << "Unable to open " << session_path << " for writing\n";
    return 1;
  }
    
  f.setf(ios::fixed);
  f << setprecision(0) << t;
  f.close();
  cerr << "Wrote session start time " << t << " to " << session_path << "\n";
  return 0;
}

double getSessionStart ()
{
  string session_path("/tmp/ros_logging_session_start");
  std::ifstream f;
  f.open(session_path.c_str());
  if (!f.is_open())
    return 0.0;
  
  double t;
  f >> t;
  return t;
}

int main (int argc, char** argv)
{
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Display help message")
    ("before,b", po::value<int>(), "Latest timestamp (unix time)")
    ("after,a", po::value<int>(), "Earliest timestamp (unix time)")
    ("min_age,n", po::value<int>(), "Minimum age (in minutes)")
    ("max_age,x", po::value<int>(), "Maximum age (in minutes)")
    ("limit_recent,l", po::value<int>(), "Show the most recent K messages")
    ("tail,t", "Requires max_age to also be provided; after initial query, "
     "continue running and display new messages as they come in")
    ("start_session,s", "Requires max_age or after to be provide.  Writes"
     " the corresponding time to /tmp/ros_logging_session_start.  It will "
     "be used by future calls to grep_log unless ignore_session is specified.")
    ("ignore_session,i", "Ignore stored session start time info")
    ("message_regex,m", po::value<string>(),
     "Regular expression that message must match")
    ("node_regex,r", po::value<string>(),
     "Regular expression that node name must match");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << endl;
    return 1;
  }

  // Build up the message filter based on command line options
  MongoLogger::MessageCriteria criteria;
  ros::WallTime now = ros::WallTime::now();
  if (vm.count("before"))
    criteria.max_time = ros::WallTime(vm["before"].as<int>());
  if (vm.count("after"))
    criteria.min_time = ros::WallTime(vm["after"].as<int>());
  if (vm.count("min_age"))
    criteria.max_time =
      now - ros::WallDuration(60*vm["min_age"].as<int>());
  if (vm.count("max_age"))
    criteria.min_time =
      now - ros::WallDuration(60*vm["max_age"].as<int>());
  if (vm.count("tail"))
  {
    if (vm.count("min_age") || vm.count("before"))
    {
      cerr << "Tail mode cannot be used with the min_age or before options"
           << endl;
      return 1;
    }
  }
  if (vm.count("limit_recent"))
    criteria.limit_recent = vm["limit_recent"].as<int>();
  if (vm.count("message_regex"))
    criteria.message_regex = vm["message_regex"].as<string>();
  if (vm.count("node_regex"))
    criteria.node_name_regex = vm["node_regex"].as<string>();
  
  if (vm.count("start_session"))
  {
    if (vm.count("max_age")==0 && vm.count("after")==0)
    {
      cerr << "--start_session requires max_age or after to be provided\n";
      return 1;
    }
    return writeSessionStart(criteria.min_time.toSec());
  }

  if (!vm.count("ignore_session"))
  {
    double t = getSessionStart();
    cerr << "Using session start time " << t << endl;
    if (t > criteria.min_time.toSec())
      criteria.min_time = ros::WallTime(t);
  }
  

  boost::scoped_ptr<MongoLogger> logger;
  

  // Connect to the db
  try
  {
    logger.reset(new MongoLogger("rosout_log"));
  }
  catch (mongo::ConnectException& e)
  {
    cerr << "Error connecting to db: '" << e.what() << "'\n";
    return 1;
  }
 
  // Do the query
  double last_receipt_time_secs=ros::WallTime::now().toSec();
  bool print_query = true;
  while (true)
  {
    BOOST_FOREACH (LogItem::ConstPtr l, logger->filterMessages(criteria,
                                                              print_query))
    {
      struct tm* time_info;
      last_receipt_time_secs = l->receipt_time.toSec();
      time_t last_receipt_time = int(last_receipt_time_secs);
      time_info = localtime(&last_receipt_time);
    
      cout << time_info->tm_year+1900 << "-" << time_info->tm_mon << "-" <<
        time_info->tm_mday << " " << time_info->tm_hour << ":" <<
        time_info->tm_min << ":" << time_info->tm_sec << " [" <<
        l->msg.name << "] " << l->msg.msg << endl;
    }
    if (vm.count("tail")==0)
      break;
    usleep(1e6);
    criteria.min_time = ros::WallTime(last_receipt_time_secs);
    print_query = false;
  }
  
  return 0;
}
