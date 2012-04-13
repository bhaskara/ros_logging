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
 * Implementation for mongo_logger.h
 *
 * \author Bhaskara Marthi
 */

#include <ros_logging/mongo_logger.h>
#include <boost/format.hpp>
#include <boost/crc.hpp>

namespace ros_logging
{

using std::string;
using rosgraph_msgs::Log;
using mongo::BSONObj;

typedef boost::shared_ptr<mongo::DBClientConnection> ConnPtr;


// Internal function to connect to the mongo instance
// Can throw a Mongo exception if connection fails
ConnPtr createConnection (const string& host, const unsigned port)
{
  const string address = (boost::format("%1%:%2%") % host % port).str();
  ConnPtr conn(new mongo::DBClientConnection());
  conn->connect(address);
  return conn;
}



MongoLogger::MongoLogger (const string& db, const string& host,
                          const unsigned port) :
  conn_(createConnection(host, port)), db_(db),
  message_coll_(db+".messages"), node_name_coll_(db+".nodes"),
  log_coll_(db+".log_items")
{
}

int computeCrc (const string& text)
{
  boost::crc_32_type res;
  res.process_bytes(text.c_str(), text.size());
  return res.checksum();
}

int MongoLogger::getCrc (const string& text, const int node_id)
{
  const int crc = computeCrc(text);
  auto_ptr<mongo::DBClientCursor> cursor = conn_->query(message_coll_, 
                             QUERY("crc" << crc << "node_id" << node_id));
  if (!cursor->more())
  {
    // We haven't seen this message before, so add it
    BSONObj item = BSON("crc" << crc << "text" << text << "node_id" << node_id);
    conn_->insert(message_coll_, item);
  }
  return crc;
}

int MongoLogger::getNodeId (const string& name)
{
  auto_ptr<mongo::DBClientCursor> cursor = 
    conn_->query(node_name_coll_, QUERY("name" << name));
  if (!cursor->more())
  {
    // We haven't seen this node before, so add it
    int id = conn_->count(node_name_coll_);
    BSONObj item = BSON("name" << name << "id" << id);
    conn_->insert(node_name_coll_, item);
    return id;
  }
  else
  {
    BSONObj item = cursor->next();
    return item.getIntField("id");
  }
}

void MongoLogger::write (const Log& msg, const string& node_name)
{
  const int node_id = getNodeId(node_name);
  const int id = getCrc(msg.msg, node_id);
  BSONObj item = BSON("crc" << id << "node" << node_id << "stamp" 
                      << msg.header.stamp.toSec());
  conn_->insert(log_coll_, item);
}


} // namespace
