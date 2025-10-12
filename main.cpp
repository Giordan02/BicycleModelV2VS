#include "src/View/main_window.h"

#include "src/Model/eqn_solver.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    //testsolver();
    
    QApplication a(argc, argv);
    MainWindow w;
    w.setTireTab();
    w.setDefaultValues();
    w.show();
    return a.exec();
    
}
