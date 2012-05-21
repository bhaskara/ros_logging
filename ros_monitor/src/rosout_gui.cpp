#include <ros_monitor/rosout_gui.h>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <iostream>

using std::string;
using boost::format;
using std::cerr;
using std::endl;

QString qstring (const format& f)
{
  return QString(f.str().c_str());
}

int DbModel::rowCount (const QModelIndex& parent) const
{
  return doubles_.size();
}

int DbModel::columnCount (const QModelIndex& parent) const
{
  return 2;
}

QVariant DbModel::headerData (int section, Qt::Orientation orientation,
                              int role) const
{
  if (role!=Qt::DisplayRole)
    return QVariant();
  if (orientation==Qt::Horizontal)
  {
  if (section==0)
    return QString("Number");
  else
    return QString("Double");
  }
  else
  {
    return section;
  }
}

QVariant DbModel::data (const QModelIndex& ind, int role) const
{
  const int r = ind.row();
  if (role!=Qt::DisplayRole) // Invalid role
    return QVariant();
  else if (!ind.isValid())
    return QVariant();
  else if (r>=static_cast<int>(doubles_.size()))
    return QVariant();
  else
  {
    std::map<int, int>::const_iterator e = doubles_.begin();
    advance(e, r);
    return ind.column() ? e->second : e->first;
  }
}

DbModel::DbModel (QObject* parent) :
  QAbstractTableModel(parent), begin_(0), row_(0)
{
  postpend(40);
}

void DbModel::postpend (const int n)
{
  const int nr = rowCount(QModelIndex());
  beginInsertRows(QModelIndex(), nr+begin_, nr+begin_+n-1);
  for (int i=nr+begin_; i<nr+begin_+n; i++)
    doubles_[i] = i*2;
  endInsertRows();
  cerr << "Row count is now " << rowCount(QModelIndex()) << endl;
  cerr << "Loop vars were " << nr << ", " << begin_ << ", " << n << endl;
}

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

// This provides a way of telling the model what we're looking at, so that it 
// can load more items if necessary.  
void DbModel::setRow (const int r)
{
  row_ = r;
}

// If the row is close to the end, load some more items
int DbModel::maybeUpdate ()
{
  int s = doubles_.size();
  if (row_+20 > s)
  {
    cerr << "Updating to max row " << s+20 << endl;
    postpend(20);
    return 0;
  }
  if (row_*4<s)
  {
    cerr << "Prepending " << s << " rows\n";
    prepend(s);
    return s;
  }
  return 0;
}


// In a separate thread, periodically load new items as necessary
void ModelUpdateThread::run()
{
  while (true)
  {
    QThread::msleep(100.0);
    if (model_)
    {
      const int dr = model_->maybeUpdate();
      if (dr>0)
      {
      const int r = view_->verticalScrollBar()->value();
      view_->verticalScrollBar()->setValue(dr+r);
      }
    }
  }
}
  
  
MainWindow::MainWindow() 
{
  items_ = new QTableView;
  setCentralWidget(items_);
  createStatusBar();
  model_ = new DbModel;
  items_->setModel(model_);
  updater_ = new ModelUpdateThread(model_, items_);
  updater_->start();
  
  connect(items_->verticalScrollBar(), SIGNAL(valueChanged(int)), this,
          SLOT(itemsScrolled(int)));
                                            
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

int main (int argv, char** args)
{
  QApplication app(argv, args);
  MainWindow w;
  w.show();
  return app.exec();
}
