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
 * Defines MongoLogger class
 *
 * \author Bhaskara Marthi
 */

#ifndef ROS_LOGGING_MONGO_LOGGER_H
#define ROS_LOGGING_MONGO_LOGGER_H

#include "log_item.h"
#include "query_results.h"
#include <boost/shared_ptr.hpp>

namespace ros_logging
{


class MongoLogger
{
public:
  
  /// Constructor
  ///
  /// \param db Name of the database within the Mongo instance.
  /// \param host Hostname of the db instance.  Defaults to localhost.
  /// \param port Port on which db is listening.  Defaults to 27019.
  /// \throws mongo::ConnectException if the connection fails, in which case the
  /// object should of course not be used further.
  MongoLogger (const std::string& db,
               const std::string& host="localhost",
               const unsigned port=27017);

  /// Append this message to the log
  /// 
  /// \param receipt_time The wall clock time at which the message was received.
  void write (const rosgraph_msgs::Log& m,
              const ros::WallTime& receipt_time);
  
  /// Used to specify criteria for filtering messages
  /// The time fields are ignored if they're equal to 0, and the strings are
  /// ignored if they're empty.
  struct MessageCriteria
  {
    ros::WallTime min_time;
    ros::WallTime max_time;
    std::string message_regex;
    std::string node_name_regex;
    unsigned limit_recent;
    unsigned min_level;
  };
  
  /// Returns messages matching criteria.
  /// The returned value is a pair of iterators and satisfies the single-pass
  /// range concept, so can be used in BOOST_FOREACH, etc.
  ResultRange filterMessages (const MessageCriteria& c,
                              bool print_query=false);
  
private:
  
  /// Return integer id based on node name.  Add to db if new.
  int getNodeId (const std::string& name);

  /// Return crc given message text.  Add to db if unseen.
  /// The node id is included to reduce the chance of collisions, so a 
  /// collision can only happen between two messages from the same node.
  int getCrc (const std::string& text, int node_id, unsigned level);
  
  /// Return vector of ids of nodes whose names match the given regex
  std::vector<int> getMatchingNodes (const std::string& regex);
  
  /// Return vector of crcs of messages matching the given regex
  std::vector<int> getMatchingMessages (const std::string& regex,
                                        const unsigned level);

  // The connection to the Mongo server
  boost::shared_ptr<mongo::DBClientConnection> conn_;
  
  // The name of the specific db within the mongo instance
  const std::string db_;
  
  // Name of collection used to intern messages
  const std::string message_coll_;

  // Name of the collection used to store node names
  const std::string node_name_coll_;

  // Name of collection used to store log entries
  const std::string log_coll_;
  
};

} // namespace

#endif // include guard
