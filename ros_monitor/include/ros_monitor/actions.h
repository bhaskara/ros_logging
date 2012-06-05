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
 * Defines ActionModel and ActionUpdateThread
 *
 * \author Bhaskara Marthi
 */

#include <QtGui>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#ifndef ROS_MONITOR_ACTIONS_H
#define ROS_MONITOR_ACTIONS_H

namespace ros_monitor
{

enum Event { GOAL, ABORTED, SUCCEEDED, PREEMPTED };

struct Action
{
  Action (const ros::WallTime& t, const std::string& name,
          const Event event) : time(t), name(name), event(event)
  {}
  
  ros::WallTime time;
  std::string name;
  Event event;
  
  typedef boost::shared_ptr<const Action> ConstPtr;
};

class ActionModel : public QAbstractTableModel
{
  Q_OBJECT

public:
  ActionModel (const ros::WallTime& start_time, QObject* parent=0);

  // Number of rows currently stored
  int rowCount (const QModelIndex& parent) const;

  // Number of columns
  int columnCount (const QModelIndex& parent) const;

  // Fetch new entries
  int fetchRecent ();

private:

  // Return the contents of cell i, j
  QVariant data (const QModelIndex& ind, int role=Qt::DisplayRole) const;
  
  // Return header information for the table
  QVariant headerData (int section, Qt::Orientation orientation,
                       int role=Qt::DisplayRole) const;
  
  std::vector<Action::ConstPtr> actions_;
  
};







} // namespace

#endif // include guard
