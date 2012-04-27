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
 * Implementation of query_results.h
 *
 * \author Bhaskara Marthi
 */

#include <ros_logging/query_results.h>

namespace ros_logging
{

using std::string;
using std::cerr;
using std::endl;
using mongo::BSONObj;
typedef boost::shared_ptr<mongo::DBClientConnection> ConnPtr;

ResultIterator::ResultIterator (ConnPtr conn, const string& message_ns,
                                const string& node_ns, const string& log_ns,
                                const mongo::Query& query) :
  // So we call query to get an auto pointer.  We then use move semantics to
  // move that auto pointer into the heap, and wrap the pointer to *that* pointer
  // in a shared pointer.  All clear?
  conn_(conn), message_ns_(message_ns), node_ns_(node_ns), log_ns_(log_ns),
  cursor_(new CursorAutoPtr(conn_->query(log_ns, query)))
{
  if ((*cursor_)->more())
    next_ = (*cursor_)->next();
}

ResultIterator::ResultIterator ()
{}

ResultIterator::ResultIterator (const ResultIterator& res) :
  conn_(res.conn_), message_ns_(res.message_ns_), node_ns_(res.node_ns_),
  log_ns_(res.log_ns_), cursor_(res.cursor_), next_(res.next_)
{
}

void ResultIterator::increment ()
{
  ROS_ASSERT(next_);
  if ((*cursor_)->more())
    next_ = (*cursor_)->nextSafe();
  else
    next_.reset();
}

LogItem::ConstPtr ResultIterator::dereference() const
{
  ROS_ASSERT(next_);
  ROS_ASSERT (next_->hasField("crc"));
  const int crc = next_->getIntField("crc");
  ROS_ASSERT (next_->hasField("node"));
  const int node = next_->getIntField("node");
  
  // Look up the actual message and node names
  mongo::Query query = QUERY("crc" << crc << "node_id" << node);
  CursorAutoPtr message_cursor = conn_->query(message_ns_, query);
  ROS_ASSERT(message_cursor->more());
  CursorAutoPtr node_cursor =
    conn_->query(node_ns_, QUERY("id" << node));
  ROS_ASSERT(node_cursor->more());

  // Reconstruct the log message
  LogItem::Ptr l(new LogItem());

  ROS_ASSERT (next_->hasField("receipt_time"));
  l->receipt_time =
    ros::WallTime(next_->getField("receipt_time").numberDouble());
 
  BSONObj msg_item = message_cursor->next();
  ROS_ASSERT(msg_item.hasField("text"));
  l->msg->msg = msg_item.getStringField("text");

  BSONObj node_item = node_cursor->next();
  ROS_ASSERT(node_item.hasField("name"));
  l->msg->name = node_item.getStringField("name");
  return l;
}

bool ResultIterator::equal (const ResultIterator& other) const
{
  // Incomplete: the only case we care about is whether we're at the end yet
  if (next_ && other.next_)
    std::cerr << "Unexpected equality check of two not-past-the-end "
              << "iterators in ResultIterator";
  return (!next_ && !other.next_);
}

} // namespace
