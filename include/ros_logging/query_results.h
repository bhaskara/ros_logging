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
 * Defines ResultIterator, an iterator over results of a query
 *
 * \author Bhaskara Marthi
 */

#ifndef ROS_LOGGING_QUERY_RESULTS_H
#define ROS_LOGGING_QUERY_RESULTS_H

#include "log_item.h"
#include <mongo/client/dbclient.h>
#include <boost/optional.hpp>

namespace ros_logging
{

// To avoid some const-correctness issues, we wrap Mongo's returned auto_ptr in
// another pointer
typedef auto_ptr<mongo::DBClientCursor> CursorAutoPtr;
typedef boost::shared_ptr<CursorAutoPtr> CursorPtr;

class ResultIterator :
    public boost::iterator_facade<ResultIterator,
                                  LogItem::ConstPtr,
                                  boost::single_pass_traversal_tag,
                                  LogItem::ConstPtr>
{
public:

  /// Create an iterator for a query
  ResultIterator (boost::shared_ptr<mongo::DBClientConnection> conn,
                  const std::string& message_ns,
                  const std::string& node_ns,
                  const std::string& log_ns,
                  const mongo::Query& query);

  /// Copy constructor.  This has move semantics, so you can't use the
  /// rhs anymore.
  ResultIterator (const ResultIterator& rhs);

  /// Default constructor (for a past-the-end iterator)
  ResultIterator ();

private:

  friend class boost::iterator_core_access;

  // Member functions needed to be an iterator
  void increment();
  LogItem::ConstPtr dereference() const;
  bool equal (const ResultIterator& other) const;
  
  boost::shared_ptr<mongo::DBClientConnection> conn_;
  const std::string message_ns_, node_ns_, log_ns_;
  CursorPtr cursor_;
  boost::optional<mongo::BSONObj> next_;

};

// A typedef for convenience
// ResultRange satisfies the single pass range concept
typedef std::pair<ResultIterator, ResultIterator> ResultRange;
    

} // namespace

#endif // include guard
