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

#include <logging_tools/mongo_logger.h>
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
using std::set;
using logging_tools::LogItem;
using logging_tools::MongoLogger;
using boost::algorithm::trim_copy;
using std::ios;
using std::setprecision;
using std::setfill;
using std::setw;
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
  string hostname;
  int port;
  desc.add_options()
    ("help,h", "Display help message")
    ("before,b", po::value<int>(), "Latest timestamp (unix time)")
    ("after,a", po::value<int>(), "Earliest timestamp (unix time)")
    ("min_age,n", po::value<int>(), "Minimum age (in minutes)")
    ("max_age,x", po::value<int>(), "Maximum age (in minutes)")
    ("tail,t", "Requires max_age to also be provided; after initial query, "
     "continue running and display new messages as they come in")
    ("list_nodes,o", "Just list all node names that have messages saved in"
     " the given time interval")
    ("start_session,s", "Requires max_age or after to be provide.  Writes"
     " the corresponding time to /tmp/ros_logging_session_start.  It will "
     "be used by future calls to grep_log unless ignore_session is specified.")
    ("db_host,z", po::value<string>(&hostname)->default_value("localhost"),
     "Hostname for the DB server.  Defaults to localhost.")
    ("db_port,p", po::value<int>(&port)->default_value(27017),
     "Port for DB server.  Defaults to 27017")
    ("ignore_session,i", "Ignore stored session start time info")
    ("message_regex,m", po::value<string>(),
     "Regular expression that message must match")
    ("verbose,v", "Show debug information")
    ("min_level,d", po::value<string>(),
     "One of d(ebug), i(nfo), w(arning), e(rror), or f(atal)")
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
  criteria.min_level=0;
  if (vm.count("min_level"))
  {
    char c = vm["min_level"].as<string>()[0];
    switch (c)
    {
    case 'd': criteria.min_level = 1; break;
    case 'i': criteria.min_level = 2; break;
    case 'w': criteria.min_level = 4; break;
    case 'e': criteria.min_level = 8; break;
    case 'f': criteria.min_level = 16; break;
    default: cerr << "Unknown value " << c << " for min_level\n";
    }
  }
  if (vm.count("tail"))
  {
    if (vm.count("min_age") || vm.count("before"))
    {
      cerr << "Tail mode cannot be used with the min_age or before options"
           << endl;
      return 1;
    }
  }
  /*
    if (vm.count("limit_recent"))
    criteria.limit_recent = vm["limit_recent"].as<int>();
  */
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
    if (t > criteria.min_time.toSec())
    {
      cerr << "Using saved session: only considering messages after " << t << endl;
      criteria.min_time = ros::WallTime(t);
    }
  }
  

  boost::scoped_ptr<MongoLogger> logger;
  

  // Connect to the db
  try
  {
    logger.reset(new MongoLogger("ros_logging", hostname, port));
    if (vm.count("verbose"))
      cerr << "Connected to db at " << hostname << ":" <<
        port << endl;
  }
  catch (mongo::ConnectException& e)
  {
    cerr << "Error connecting to db: '" << e.what() << "'\n";
    return 1;
  }
  
  // Print color output?
  string term(getenv("TERM"));
  bool color=false;
  if (term=="xterm" || term=="eterm-color")
    color = true;
  
 
  // Do the query and print results
  double last_receipt_time_secs=ros::WallTime::now().toSec();
  bool print_query = vm.count("verbose")>0;
  set<string> seen_nodes;

  // Outer loop to handle the -t case.  Will only run once if -t not specified.
  while (true)
  {

    // Inner loop over messages satisfying query.  
    BOOST_FOREACH (LogItem::ConstPtr l, logger->filterMessages(criteria,
                                                               print_query))
    {
      // Special case where we're just listing nodes
      if (vm.count("list_nodes")>0)
      {
        if (seen_nodes.find(l->msg->name)==seen_nodes.end())
        {
          cout << l->msg->name << endl;
          seen_nodes.insert(l->msg->name);
        }
      }

      // Normal case where we're printing messages
      else
      {
        struct tm* time_info;
        last_receipt_time_secs = l->receipt_time.toSec();
        time_t last_receipt_time = int(last_receipt_time_secs);
        time_info = localtime(&last_receipt_time);
    
        cout << time_info->tm_year+1900 << "-" << setfill('0') << setw(2) <<
          time_info->tm_mon << "-" << setfill('0') << setw(2) <<
          time_info->tm_mday << " " << setfill('0') << setw(2) <<
          time_info->tm_hour << ":" << setfill('0') << setw(2) <<
          time_info->tm_min << ":" << setfill('0') << setw(2) << 
          time_info->tm_sec;
        if (color)
          cout << "\033[1;31m";
        cout << " [" << l->msg->name << "] ";
        if (color)
          cout << "\033[0m";
        cout << l->msg->msg << endl;
      }
    }
    if (vm.count("tail")==0)
      break;
    usleep(1e6);
    criteria.min_time = ros::WallTime(last_receipt_time_secs);
    print_query = false;
  }
  
  return 0;
}
