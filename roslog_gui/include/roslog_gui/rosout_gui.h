#include <QtGui>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow ();
  
private slots:
  void itemsScrolled(int i);

private:
  
  void createStatusBar();

  QTableView *items_;
};
