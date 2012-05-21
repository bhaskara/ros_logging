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
 * Ros monitor gui
 *
 * \author Bhaskara Marthi
 */

#ifndef ROS_MONITOR_ROSOUT_GUI_H
#define ROS_MONITOR_ROSOUT_GUI_H

#include <ros_monitor/display_delegates.h>
#include <QtGui>
#include <logging_tools/mongo_logger.h>

namespace ros_monitor
{

// Model that fetches entries from the database and gives them to the
// view on demand. 
//
// We store a set of rows from the db, and prepend and postpend this set 
// as necessary
class DbModel : public QAbstractTableModel
{
  Q_OBJECT

public:
  DbModel (const ros::WallTime& start_time, QObject* parent=0);

  // Set the row_ field.  This may trigger maybeUpdate below to fetch more data.
  void setRow(int i);
  
  // Periodically fetch recent data; return amount by which view must be scrolled
  int maybeUpdate();

  // Return the number of rows in the stored table 
  int rowCount (const QModelIndex& parent) const;
  
  // Always return 2
  int columnCount (const QModelIndex& parent) const;
  
  // Fetch older entries
  int prependSince (const ros::WallTime& t);
  
private:

  
  // Return the contents of cell i, j
  QVariant data (const QModelIndex& ind, int role=Qt::DisplayRole) const;
  
  // Return header information for the table
  QVariant headerData (int section, Qt::Orientation orientation,
                       int role=Qt::DisplayRole) const;
  
  
  // Fetch new entries 
  int fetchRecent ();

  // Fetch up to n new entries at the beginning of the table
  /*
  void prepend (int n);
  */
  
  logging_tools::MongoLogger db_;
  
  // Subset of the db that we have loaded into memory and display using the gui
  std::vector<logging_tools::LogItem::ConstPtr> log_items_;
  
  // We'll fetch messages received after this time
  ros::WallTime start_time_;
  
  // The row that is currently at the top of the window
  int row_;
  
};

class ModelUpdateThread;

// The main application consists of a table view backed by a model, 
// and a thread that makes the model update itself
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow ();
  ~MainWindow ();

  void update();
  virtual QSize sizeHint();

  
private slots:
  void itemsScrolled(int i);
  void setTailMode (bool m);
  void sliderMoved ();
  void updateStartTime(const QDateTime& t);

private:
  
  void createStatusBar();

  QTableView* view_;
  DbModel* model_;
  TimestampDelegate* timestamp_display_;
  ModelUpdateThread* updater_;
  QRadioButton* tail_button_;
  QDateTimeEdit* start_time_input_;

  bool tail_mode_;
};

// A thread that causes the model to fetch more rows as required in the
// background
class ModelUpdateThread : public QThread
{
  Q_OBJECT

public:
  ModelUpdateThread (MainWindow* main_window) :
    main_window_(main_window)
  {}
  void run ();

private:

  MainWindow* main_window_;
};

} // namespace

#endif 
