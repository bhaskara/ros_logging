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

namespace ros_monitor
{

typedef std::vector<Action::ConstPtr> Actions;

int ActionModel::rowCount (const QModelIndex& parent) const
{
  return actions_.size();
}

int ActionModel::columnCount (const QModelIndex& parent) const
{
  return 3;
}

ActionModel::ActionModel (const ros::WallTime& start_time,
                          QObject* parent) :
  QAbstractTableModel(parent)
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


typedef std::map<Event, std::string> EventNames;

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
  // Todo actually fetch
  Actions new_items;
  for (unsigned i=0; i<2; i++)
  {
    Action::ConstPtr a(new Action(ros::WallTime(42.4242), "foo", SUCCEEDED));
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
