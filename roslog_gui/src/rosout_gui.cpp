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

MainWindow::MainWindow()
{
  items_ = new QTableView;
  setCentralWidget(items_);
  createStatusBar();
  
  for (unsigned i=0; i<100; i++)
    items_->addItem(QString(boost::lexical_cast<string>(i).c_str()));
  
  connect(items_->verticalScrollBar(), SIGNAL(valueChanged(int)), this,
          SLOT(itemsScrolled(int)));
                                            
}

void MainWindow::itemsScrolled (int i)
{
  format f("Scrolled to %1%");
  statusBar()->showMessage(qstring(f % i));
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
