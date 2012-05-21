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
    return section;
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

void DbModel::prepend (const size_t n)
{
  lt::MongoLogger::MessageCriteria c;
  c.min_time = ros::WallTime();
  c.max_time = log_items_.empty() ?
    ros::WallTime() :
    (*log_items_.begin())->receipt_time;
  c.limit_recent = n;
  lt::ResultRange r = db_.filterMessages(c);
  LogItems new_items(r.first, r.second);
  beginResetModel();
  log_items_.insert(log_items_.begin(), new_items.begin(), new_items.end());
  endResetModel();
}

int DbModel::maybeUpdate ()
{
  fetchRecent();
  return 0;
}



// In a separate thread, periodically load new items as necessary
void ModelUpdateThread::run()
{
  while (true)
  {
    QThread::msleep(100.0);
    main_window_->update();
    /*
    if (model_)
    {
      model_->maybeUpdate();
      if (tailing_)
        view_->scrollToBottom();
      const int row = view_->verticalScrollBar()->value();
      const int nr = model_->rowCount(QModelIndex());
      const bool tailing = (row==nr-1);
      const int dr = model_->maybeUpdate();
      const int scroll_by = ((dr>=0) || tailing) ? dr : 0;
      if (scroll_by>0)
      {
        const int r = view_->verticalScrollBar()->value();
        cerr << "Scrolling to " << scroll_by+r << "\n";
        view_->verticalScrollBar()->setValue(scroll_by+r);
      }
    }
    */
  }
}

void MainWindow::update ()
{
  model_->maybeUpdate();
  if (tail_mode_)
    items_->scrollToBottom();
}
  
  
MainWindow::MainWindow() : tail_mode_(true)
{
  items_ = new QTableView;
  setCentralWidget(items_);
  createStatusBar();
  model_ = new DbModel(ros::WallTime::now());
  items_->setModel(model_);
  timestamp_display_ = new TimestampDelegate;
  items_->setItemDelegateForColumn(0, timestamp_display_);
  items_->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
  QToolBar* toolbar = new QToolBar(this);
  tail_button_ = new QRadioButton("Tail messages", toolbar);
  tail_button_->setChecked(true);
  connect(tail_button_, SIGNAL(toggled(bool)), this,
          SLOT(setTailMode(bool)));
  toolbar->addWidget(tail_button_);
  addToolBar(toolbar);
  updater_ = new ModelUpdateThread(this);
  updater_->start();
  
  connect(items_->verticalScrollBar(), SIGNAL(valueChanged(int)), this,
          SLOT(itemsScrolled(int)));
  connect(items_->verticalScrollBar(), SIGNAL(sliderMoved(int)), this,
          SLOT(sliderMoved()));
                                            
}



MainWindow::~MainWindow()
{
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
