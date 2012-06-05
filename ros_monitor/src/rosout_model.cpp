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
 * Implementation of rosout_model.h
 *
 * \author Bhaskara Marthi
 */

#include <ros_monitor/rosout_model.h>

namespace ros_monitor
{

namespace lt=logging_tools;

using boost::format;

typedef std::vector<lt::LogItem::ConstPtr> LogItems;

int DbModel::rowCount (const QModelIndex& parent) const
{
  return log_items_.size();
}

int DbModel::columnCount (const QModelIndex& parent) const
{
  return 3;
}

QVariant DbModel::headerData (int section, Qt::Orientation orientation,
                              int role) const
{
  if (role!=Qt::DisplayRole)
    return QVariant();
  if (orientation==Qt::Horizontal)
  {
    if (section==0)
      return QString("Time");
    else if (section==1)
      return QString("Node");
    else
      return QString("Message");
  }
  else
  {
    return QVariant();
  }
}

// Return the actual contents of a cell in the table
QVariant DbModel::data (const QModelIndex& ind, int role) const
{
  // First, a bunch of validity checks
  const int r = ind.row();
  if (role!=Qt::DisplayRole) // Invalid role
    return QVariant();
  else if (!ind.isValid())
    return QVariant();
  else if (r>=static_cast<int>(log_items_.size()))
    return QVariant();
  
  // Now the nominal case
  else
  {
    LogItems::const_iterator e = log_items_.begin();
    advance(e, r);
    lt::LogItem::ConstPtr item = *e;
    if (ind.column()==0)
      return item->receipt_time.toSec();
    else if (ind.column()==1)
      return item->msg->name.c_str();
    else
      return item->msg->msg.c_str();
  }
}



DbModel::DbModel (const ros::WallTime& start_time, QObject* parent) :
  QAbstractTableModel(parent), db_("ros_logging"), start_time_(start_time), row_(0)
{
}

int DbModel::fetchRecent ()
{
  lt::MongoLogger::MessageCriteria c;
  c.min_time = log_items_.empty() ?
    start_time_ :
    log_items_.back()->receipt_time;
  c.max_time = ros::WallTime();
  lt::ResultRange r = db_.filterMessages(c, false);
  LogItems new_items(r.first, r.second);
  const size_t num_rows = log_items_.size();


  const size_t new_rows = new_items.size();
  beginInsertRows(QModelIndex(), num_rows, num_rows+new_rows-1);
  log_items_.insert(log_items_.end(), new_items.begin(), new_items.end());
  endInsertRows();
  return new_rows;
}

/*
  void DbModel::prepend (const int n)
  {
  cerr << "In prepend\n";
  beginResetModel();
  for (int i=begin_-n; i<begin_; i++)
  doubles_[i] = i*2;
  endResetModel();
  begin_ -= n;
  row_ += n;
  }
*/

// This provides a way of telling the model what we're looking at, so that it 
// can load more items if necessary.  
void DbModel::setRow (const int r)
{
  row_ = r;
}

int DbModel::prependSince (const ros::WallTime& start)
{
  lt::MongoLogger::MessageCriteria c;
  c.min_time = start;
  c.max_time = log_items_.empty() ?
    ros::WallTime() :
    (*log_items_.begin())->receipt_time;
  lt::ResultRange r = db_.filterMessages(c, true);
  LogItems new_items(r.first, r.second);
  beginResetModel();
  log_items_.insert(log_items_.begin(), new_items.begin(), new_items.end());
  endResetModel();
  return static_cast<int>(new_items.size());
}




} // namespace
