#include <ros_monitor/ros_monitor.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <iostream>

namespace ros_monitor
{

namespace lt=logging_tools;

using std::string;
using std::vector;
using boost::format;
using std::cerr;
using std::endl;
using boost::optional;

typedef boost::mutex::scoped_lock Lock;
typedef vector<lt::LogItem::ConstPtr> LogItems;


// Utility conversion function
QString qstring (const format& f)
{
  return QString(f.str().c_str());
}


ModelUpdateThread::~ModelUpdateThread ()
{
  done_ = true;
  this->wait();
}

ActionUpdateThread::~ActionUpdateThread ()
{
  done_ = true;
  this->wait();
}

// This is supposed to fetch new items from the db.  But to avoid blocking the 
// gui thread, we just set a variable that causes them to be fetched in the
// update thread.  
void MainWindow::updateStartTime (const QDateTime& t)
{
  Lock l(mutex_);
  updated_start_time_ = t;
}


// In a separate thread, periodically load new items as necessary
// Note this is not well organized.  Instead of having updateRosout/action in
// MainWindow, each pane should inherit from QWidget and have its own internal
// update method.
void ModelUpdateThread::run()
{
  while (!done_)
  {
    QThread::msleep(100.0);
    main_window_->updateRosout();
  }
}


// Load new actions as necessary
void ActionUpdateThread::run()
{
  while (!done_)
  {
    QThread::msleep(1000.0);
    main_window_->updateActions();
  }
}


void MainWindow::updateActions()
{
  // Fetch new items
  action_model_->fetchRecent();

  // For now we're always tailing
  action_view_->scrollToBottom();
}


void MainWindow::updateRosout()
{
  // Fetch new items
  rosout_model_->fetchRecent();
  
  // Determine whether the start time has been changed, and if so
  // take note and atomically reset it
  optional<QDateTime> new_start;
  {
    Lock l(mutex_);
    new_start = updated_start_time_;
    updated_start_time_.reset();
  }
  
  // If start time was changed, fetch the old messages and scroll so that the view
  // doesn't shift
  if (new_start)
  {
    const int r = rosout_view_->verticalScrollBar()->value();
    const int nr = rosout_model_->prependSince(ros::WallTime(new_start->toTime_t()));
    rosout_view_->verticalScrollBar()->setValue(r+nr);
  }
  
  // If we're tailing, scroll as new messages come in
  else if (rosout_tail_mode_)
    rosout_view_->scrollToBottom();
}
  
  
MainWindow::MainWindow() : rosout_tail_mode_(true) 
{
  tabs_ = new QTabWidget(this);
  setCentralWidget(tabs_);
  
  QWidget* rosout_tab = new QWidget;
  tabs_->addTab(rosout_tab, "Rosout");
  
  // rosout view
  rosout_view_ = new QTableView();
  rosout_model_ = new DbModel(ros::WallTime::now());
  rosout_view_->setModel(rosout_model_);
  TimestampDelegate* timestamp_display = new TimestampDelegate(rosout_view_);
  rosout_view_->setItemDelegateForColumn(0, timestamp_display);
  rosout_view_->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
  
  // Rosout toolbar
  QToolBar* rosout_toolbar = new QToolBar;
  tail_button_ = new QRadioButton("Tail messages", rosout_toolbar);
  tail_button_->setChecked(true);
  connect(tail_button_, SIGNAL(toggled(bool)), this,
          SLOT(setTailMode(bool)));
  rosout_toolbar->addWidget(tail_button_);
  QDateTime t;
  t.setTime_t(ros::WallTime::now().toSec());
  QDateTimeEdit* start_time_input = new QDateTimeEdit(t, rosout_toolbar);
  connect(start_time_input, SIGNAL(dateTimeChanged(const QDateTime&)),
          this, SLOT(updateStartTime(const QDateTime&)));
  rosout_toolbar->addWidget(start_time_input);

  // Rosout vertical layout
  QVBoxLayout* rosout_layout = new QVBoxLayout(rosout_tab);
  rosout_layout->addWidget(rosout_toolbar);
  rosout_layout->addWidget(rosout_view_);
  rosout_tab->setLayout(rosout_layout);
  
  // Update thread for rosout
  ModelUpdateThread* rosout_updater = new ModelUpdateThread(this);
  rosout_updater->start();
  
  connect(rosout_view_->verticalScrollBar(), SIGNAL(valueChanged(int)), this,
          SLOT(itemsScrolled(int)));
  connect(rosout_view_->verticalScrollBar(), SIGNAL(sliderMoved(int)), this,
          SLOT(sliderMoved()));
  
  // Action tab
  QWidget* action_tab = new QWidget;
  tabs_->addTab(action_tab, "Actions");
  
  // Action view
  action_view_ = new QTableView();
  action_model_ = new ActionModel(ros::WallTime::now());
  action_view_->setModel(action_model_);
  TimestampDelegate* action_timestamp_display =
    new TimestampDelegate(action_view_);
  action_view_->setItemDelegateForColumn(0, action_timestamp_display);

  // Action layout
  QVBoxLayout* action_layout = new QVBoxLayout(action_tab);
  action_layout->addWidget(action_view_);
  action_tab->setLayout(action_layout);
  
  // Action view update thread
  ActionUpdateThread* action_updater = new ActionUpdateThread(this);
  action_updater->start();
                                              
  resize(sizeHint());
}



MainWindow::~MainWindow()
{
}

QSize MainWindow::sizeHint ()
{
  return QSize(800, 600);
}

void MainWindow::setTailMode (const bool checked)
{
  rosout_tail_mode_ = checked;
}

void MainWindow::sliderMoved ()
{
  tail_button_->setChecked(false);
}
  
void MainWindow::itemsScrolled (int i)
{
  format f("Scrolled to %1%");
  statusBar()->showMessage(qstring(f % i));
  rosout_model_->setRow(i);
}

void MainWindow::createStatusBar()
{
  statusBar()->showMessage(tr("Ready"));
}


} // namespace

int main (int argv, char** args)
{
  QApplication app(argv, args);
  ros_monitor::MainWindow w;
  w.show();
  return app.exec();
}
