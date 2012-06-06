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
 * Implementation of actions.h
 *
 * \author Bhaskara Marthi
 */

#include <ros_monitor/actions.h>
#include <mongo/client/dbclient.h>

namespace ros_monitor
{

using std::string;
typedef std::vector<Action::ConstPtr> Actions;
using std::cerr;
using std::endl;

int ActionModel::rowCount (const QModelIndex& parent) const
{
  return actions_.size();
}

int ActionModel::columnCount (const QModelIndex& parent) const
{
  return 3;
}

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


ActionModel::ActionModel (const ros::WallTime& start_time,
                          QObject* parent) :
  QAbstractTableModel(parent), start_time_(start_time.toSec()),
  conn_(createConnection("localhost", 27017))
{}

QVariant ActionModel::headerData (int section, Qt::Orientation orientation,
                                  int role) const
{
  if (role!=Qt::DisplayRole)
    return QVariant();
  if (orientation!=Qt::Horizontal)
    return QVariant();

  switch (section)
  {
  case 0:
    return QString("Time");

  case 1:
    return QString("Name");
  default:
    return QString("Event");
  }
}


typedef std::map<Event, string> EventNames;

EventNames initializeEventNames ()
{
  EventNames names;
  names[GOAL] = "goal";
  names[ABORTED] = "aborted";
  names[SUCCEEDED] = "succeeded";
  names[PREEMPTED] = "preempted";
  return names;
}


// Return the actual contents of a cell in the table
QVariant ActionModel::data (const QModelIndex& ind, const int role) const
{
  static EventNames event_names = initializeEventNames();

  // First, a bunch of validity checks
  const int r = ind.row();
  if (role!=Qt::DisplayRole) // Invalid role
    return QVariant();
  else if (!ind.isValid())
    return QVariant();
  else if (r>=static_cast<int>(actions_.size()))
    return QVariant();
  
  // Nominal case
  Action::ConstPtr a = actions_.at(r);
  switch (ind.column())
  {
  case 0:
    return a->time.toSec();
  case 1:
    return a->name.c_str();
  default:
    return event_names[a->event].c_str();
  }
}

// Fetch recent items
int ActionModel::fetchRecent ()
{
  const double min_time = actions_.empty() ? start_time_ :
    actions_.back()->time.toSec()+1e-8;
  boost::format query_fmt("{ time: {$gt : %.9f} }");
  const std::string query_string = (query_fmt % min_time).str();
  //cerr << "Action query is " << query_string << endl;
  mongo::Query q = mongo::fromjson(query_string);
  std::auto_ptr<mongo::DBClientCursor> cursor = conn_->query("ros_logging.actions", q);
  Actions new_items;
  const string goal_name("goal");
  while (cursor->more())
  {
    mongo::BSONObj item = cursor->next();
    const ros::WallTime t(item.getField("time").Double());
    Event e;
    if (item.getStringField("type")==goal_name)
      e = GOAL;
    else
    {
      const int s = item.getIntField("status");
      if (s==2)
        e = PREEMPTED;
      else if (s==3)
        e = SUCCEEDED;
      else
        e = ABORTED;
      
    }
      
    mongo::OID action_id = item.getField("action_id").OID();
    mongo::Query q2 = BSON("_id" << action_id);
    //cerr << "About to do query " << q2.toString() << endl;
    std::auto_ptr<mongo::DBClientCursor> c2 =
      conn_->query("ros_logging.action_logging_action_types", q2);
    
    mongo::BSONObj b = c2->next();
    const string name = b.getStringField("name");
    Action::Ptr a(new Action(t, name, e));
    new_items.push_back(a);
  }
  
  const size_t num_rows = actions_.size();
  const size_t new_rows = new_items.size();
  beginInsertRows(QModelIndex(), num_rows, num_rows+new_rows-1);
  actions_.insert(actions_.end(), new_items.begin(), new_items.end());
  endInsertRows();
  return new_rows;
}

} // namespace
