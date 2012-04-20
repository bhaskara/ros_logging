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
#include <boost/foreach.hpp>

namespace ros_logging
{

using std::string;
using std::cerr;
using std::endl;
using rosgraph_msgs::Log;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::BSONArrayBuilder;

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
  // Make sure various indices are set up
  conn_->ensureIndex(message_coll_, BSON("crc" << 1));
  conn_->ensureIndex(message_coll_, BSON("node_id" << 1));
  conn_->ensureIndex(log_coll_, BSON("receipt_time" << 1));
  conn_->ensureIndex(log_coll_, BSON("crc" << 1));
  conn_->ensureIndex(log_coll_, BSON("node" << 1));
  conn_->ensureIndex(node_name_coll_, BSON("name" << 1));
  conn_->ensureIndex(node_name_coll_, BSON("id" << 1));
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
  CursorAutoPtr cursor = conn_->query(message_coll_, 
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
  CursorAutoPtr cursor = 
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
    ROS_ASSERT(item.hasField("id"));
    return item.getIntField("id");
  }
}

vector<int> MongoLogger::getMatchingNodes (const string& regex)
{
  BSONObj query = BSONObjBuilder().appendRegex("name", regex).obj();
  CursorAutoPtr cursor = conn_->query(node_name_coll_, query);
  vector<int> nodes;
  while (cursor->more())
  {
    BSONObj item = cursor->next();
    ROS_ASSERT(item.hasField("id"));
    nodes.push_back(item.getIntField("id"));
  }
  return nodes;
}

vector<int> MongoLogger::getMatchingMessages (const string& regex)
{
  boost::format query_string("{ text : /%1%/i }");
  BSONObj query = mongo::fromjson((query_string % regex).str());
  CursorAutoPtr cursor = conn_->query(message_coll_, query);
  vector<int> messages;
  while (cursor->more())
  {
    BSONObj item = cursor->next();
    ROS_ASSERT(item.hasField("crc"));
    messages.push_back(item.getIntField("crc"));
  }
  return messages;
}

void MongoLogger::write (const Log& l,
                         const ros::WallTime& receipt_time)
{
  const int node_id = getNodeId(l.name);
  const int id = getCrc(l.msg, node_id);
  BSONObj item = BSON("crc" << id << "node" << node_id << "receipt_time" 
                      << receipt_time.toSec() << "level" << l.level);
  conn_->insert(log_coll_, item);
}

ResultRange MongoLogger::filterMessages (const MessageCriteria& c)
{
  BSONObjBuilder builder;
  if (!c.message_regex.empty())
  {
    BSONArrayBuilder id_builder;
    vector<int> ids = getMatchingMessages(c.message_regex);
    BOOST_FOREACH (const int id, ids)
      id_builder.append(id);
    BSONObjBuilder sub;
    sub.append("$in", id_builder.arr());
    builder.append("crc", sub.obj());
  }
  if (!c.node_name_regex.empty())
  {
    BSONArrayBuilder id_builder;
    vector<int> ids = getMatchingNodes(c.node_name_regex);
    BOOST_FOREACH (const int id, ids)
      id_builder.append(id);
    BSONObjBuilder sub;
    sub.append("$in", id_builder.arr());
    builder.append("node", sub.obj());
  }
  
  const double max_time = c.max_time.toSec();
  if (max_time > 0.0)
  {
    BSONObjBuilder sub;
    sub.append("$lte", max_time);
    builder.append("receipt_time", sub.obj());
  }

  const double min_time = c.min_time.toSec();
  if (min_time > 0.0)
  {
    BSONObjBuilder sub;
    sub.append("$gte", min_time);
    builder.append("receipt_time", sub.obj());
  }
  BSONObjBuilder sub;
  sub.append("receipt_time", 1);
  BSONObjBuilder sorted_builder;
  sorted_builder.append("query", builder.obj());
  sorted_builder.append("orderby", sub.obj());
  BSONObj query = sorted_builder.obj();
  cerr << "Final query is: " << query.toString() << endl;
  ResultIterator iter(conn_, message_coll_, node_name_coll_, log_coll_, query);
  return ResultRange(iter, ResultIterator());
}

} // namespace
