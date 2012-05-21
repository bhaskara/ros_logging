#include <QtGui>

class DbModel : public QAbstractTableModel
{
  Q_OBJECT

public:
  DbModel (QObject* parent=0);

  void setRow(int i);
  int maybeUpdate();

private:

  int rowCount (const QModelIndex& parent) const;
  int columnCount (const QModelIndex& parent) const;
  QVariant data (const QModelIndex& ind, int role=Qt::DisplayRole) const;
  QVariant headerData (int section, Qt::Orientation orientation,
                       int role=Qt::DisplayRole) const;
  void postpend (int n);
  void prepend (int n);
  
  std::map<int, int> doubles_;
  int begin_;
  int row_;
  
};

class ModelUpdateThread : public QThread
{
  Q_OBJECT

public:
  ModelUpdateThread (DbModel* model,
                     QTableView* view) : model_(model), view_(view) {}
  void run ();

private:

  DbModel* model_;
  QTableView* view_;
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow ();
  
private slots:
  void itemsScrolled(int i);

private:
  
  void createStatusBar();

  QTableView* items_;
  DbModel* model_;
  ModelUpdateThread* updater_;
};
