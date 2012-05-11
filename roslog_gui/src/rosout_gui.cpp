#include <roslog_gui/rosout_gui.h>
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
  return squares_.size();
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
    return QString("Square");
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
  else if (r>=(int)squares_.size())
    return QVariant();
  else
  {
    std::map<int, int>::const_iterator e = squares_.begin();
    advance(e, r);
    return ind.column() ? e->second : e->first;
  }
}

DbModel::DbModel (QObject* parent) :
  QAbstractTableModel(parent), row_(0)
{
  postpend(0, 50);
}

void DbModel::postpend (const int a, const int b)
{
  const int nr = rowCount(QModelIndex());
  beginInsertRows(QModelIndex(), nr, nr+b-a);
  for (int i=a; i<b; i++)
    squares_[i]=i*i;
  endInsertRows();
}

void DbModel::setRow (const int r)
{
  row_ = r;
}

void DbModel::maybeUpdate ()
{
  int s = squares_.size();
  if (row_+20 > s)
  {
    cerr << "Updating to max row " << s+20 << endl;
    postpend(s, s+20);
  }
}



void ModelUpdateThread::run()
{
  while (true)
  {
    QThread::msleep(100.0);
    if (model_)
      model_->maybeUpdate();
  }
}
  
  
MainWindow::MainWindow() 
{
  items_ = new QTableView;
  setCentralWidget(items_);
  createStatusBar();
  model_ = new DbModel;
  items_->setModel(model_);
  updater_ = new ModelUpdateThread(model_);
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
