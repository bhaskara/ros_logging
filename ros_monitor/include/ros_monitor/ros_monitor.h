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

#ifndef ROS_MONITOR_ROS_MONITOR_H
#define ROS_MONITOR_ROS_MONITOR_H

#include <ros_monitor/display_delegates.h>
#include <ros_monitor/rosout_model.h>
#include <ros_monitor/actions.h>
#include <QtGui>
#include <boost/optional.hpp>

namespace ros_monitor
{

// The main window has tabs for rosout and actions.
// Each tab has a db model and corresponding view, as well as a toolbar.
// There are background update threads that fetch new items for each one.
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow ();
  ~MainWindow ();

  virtual QSize sizeHint();
  
  void updateRosout();

  void updateActions();
                      
private slots:
  void itemsScrolled(int i);
  void setTailMode (bool m);
  void sliderMoved ();
  void updateStartTime(const QDateTime& t);

private:
  
  void createStatusBar();
  

  boost::mutex mutex_;
  
  // Set of tabs
  QTabWidget* tabs_;

  // The table view for rosout, which is accessed in the update thread
  QTableView* rosout_view_;

  // Db model for rosout, accessed in the update thread and the scroll slot
  DbModel* rosout_model_;
  
  // The tail mode button for rosout, accessed in the sliderMoved slot
  QRadioButton* tail_button_;
  
  // Used to communicate to update thread that the start time has been changed
  boost::optional<QDateTime> updated_start_time_;

  // Db model for actions, accessed in the update thread
  ActionModel* action_model_;

  // Table view for actions, accessed in the update thread
  QTableView* action_view_;

  // Are we in tail mode, where we scroll with new messages?
  bool rosout_tail_mode_;
  
};

// A thread that causes the model to fetch more rows as required in the
// background
class ModelUpdateThread : public QThread
{
  Q_OBJECT

public:
  ModelUpdateThread (MainWindow* main_window) :
    QThread(main_window), main_window_(main_window), done_(false)
  {}

  ~ModelUpdateThread();
  
  void run ();

private:

  MainWindow* main_window_;
  bool done_;
};


// A thread that causes the action model to fetch more rows as required in the
// background
class ActionUpdateThread : public QThread
{
  Q_OBJECT

public:
  ActionUpdateThread (MainWindow* main_window) :
    QThread(main_window), main_window_(main_window), done_(false)
  {}
  
  ~ActionUpdateThread();


  void run ();

private:

  MainWindow* main_window_;
  bool done_;
};


} // namespace

#endif 
