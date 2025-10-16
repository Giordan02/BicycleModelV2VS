#include "src/View/main_window.h"

#include "src/Model/eqn_solver.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    //testsolver();

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
    Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
    
    QApplication a(argc, argv);
    MainWindow w;
    w.setTireTab();
    w.setDefaultValues();
    w.show();
    return a.exec();
    
    
}
