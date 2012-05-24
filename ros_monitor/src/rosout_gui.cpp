#include <ros_monitor/rosout_gui.h>
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


QString qstring (const format& f)
{
  return QString(f.str().c_str());
}

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

lt::LogItem fakeLogItem (const int i)
{
  lt::LogItem l;
  
  l.receipt_time = ros::WallTime::now();
  l.msg.reset(new rosgraph_msgs::Log());
  l.msg->name = (i%2) ? "foo" : "bar";
  format f("Message %1%");
  l.msg->msg = (f % i).str();
  return l;
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


// This is supposed to fetch new items from the db.  But to avoid blocking the 
// gui thread, we just set a variable that causes them to be fetched in the
// update thread.  
void MainWindow::updateStartTime (const QDateTime& t)
{
  Lock l(mutex_);
  updated_start_time_ = t;
}


// In a separate thread, periodically load new items as necessary
void ModelUpdateThread::run()
{
  while (true)
  {
    QThread::msleep(100.0);
    main_window_->update();
  }
}

void MainWindow::update ()
{
  // Fetch new items
  model_->fetchRecent();
  
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
    const int r = view_->verticalScrollBar()->value();
    const int nr = model_->prependSince(ros::WallTime(new_start->toTime_t()));
    view_->verticalScrollBar()->setValue(r+nr);
  }
  
  // If we're tailing, scroll as new messages come in
  else if (tail_mode_)
    view_->scrollToBottom();
}
  
  
MainWindow::MainWindow() : tail_mode_(true)
{
  view_ = new QTableView;
  setCentralWidget(view_);
  createStatusBar();
  model_ = new DbModel(ros::WallTime::now());
  view_->setModel(model_);
  timestamp_display_ = new TimestampDelegate;
  view_->setItemDelegateForColumn(0, timestamp_display_);
  view_->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
  
  QToolBar* toolbar = new QToolBar(this);
  tail_button_ = new QRadioButton("Tail messages", toolbar);
  tail_button_->setChecked(true);
  connect(tail_button_, SIGNAL(toggled(bool)), this,
          SLOT(setTailMode(bool)));
  toolbar->addWidget(tail_button_);
  
  QDateTime t;
  t.setTime_t(ros::WallTime::now().toSec());
  start_time_input_ = new QDateTimeEdit(t, toolbar);
  connect(start_time_input_, SIGNAL(dateTimeChanged(const QDateTime&)),
          this, SLOT(updateStartTime(const QDateTime&)));
  toolbar->addWidget(start_time_input_);
  
  addToolBar(toolbar);
  updater_ = new ModelUpdateThread(this);
  updater_->start();
  
  connect(view_->verticalScrollBar(), SIGNAL(valueChanged(int)), this,
          SLOT(itemsScrolled(int)));
  connect(view_->verticalScrollBar(), SIGNAL(sliderMoved(int)), this,
          SLOT(sliderMoved()));
                                            
  resize(sizeHint());
}



MainWindow::~MainWindow()
{
}

QSize MainWindow::sizeHint ()
{
  return QSize(640, 400);
}

void MainWindow::setTailMode (const bool checked)
{
  tail_mode_ = checked;
}

void MainWindow::sliderMoved ()
{
  tail_button_->setChecked(false);
}
  
void MainWindow::itemsScrolled (int i)
{
  format f("Scrolled to %1%");
  statusBar()->showMessage(qstring(f % i));
  model_->setRow(i);
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
