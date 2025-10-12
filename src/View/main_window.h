#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "src/controller/simulation_inputs.h"
#include <QComboBox>

QT_BEGIN_NAMESPACE
namespace Ui {

/**
 * @class MainWindow
 * @brief Main application window for the Vehicle Optimization GUI.
 * 
 * This class provides the user interface to:
 * - Configure vehicle, tire, and solver parameters.
 * - Run genetic algorithm optimization.
 * - Display optimization progress and results.
 * - Save and manage custom tire models.
 * 
 * It connects user actions from the GUI (buttons, combo boxes, etc.)
 * to simulation and optimization logic.
 */
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void setTireTab();
    void setDefaultValues();
    void defaultTireDatabase();

private slots:

    // Declaration of Buttons and when Labels are set from the GUI
    void on_radiusInput_editingFinished();

    void on_massInput_editingFinished();

    void on_aInput_editingFinished();

    void on_bInput_editingFinished();

    void on_CdInput_editingFinished();

    void on_frontalAreaInput_editingFinished();

    void on_maxIterationInput_editingFinished();

    void on_Eqn1TolInput_editingFinished();

    void on_Eqn2TolInput_editingFinished();
    
    void on_Eqn3TolInput_editingFinished();

    void on_Eqn4TolInput_editingFinished();

    void on_Eqn5TolInput_editingFinished();

    void on_Eqn6TolInput_editingFinished();

    void on_Eqn7TolInput_editingFinished();

    void on_genNumInput_editingFinished();

    void on_minDeltaInput_editingFinished();

    void on_PopSizeInput_editingFinished();

    void on_maxDeltaInput_editingFinished();

    void on_minAlphafInput_editingFinished();

    void on_maxAlphafInput_editingFinished();

    void on_minAlpharInput_editingFinished();

    void on_maxAlpharInput_editingFinished();

    void on_minKappafInput_editingFinished();

    void on_maxKappafInput_editingFinished();

    void on_minKapparInput_editingFinished();

    void on_maxKapparInput_editingFinished();

    void on_plotTireForcesButton_clicked();

    void on_IASlider_valueChanged(int value);

    void on_normalLoadSlider_valueChanged(int value);

    void on_calculateButton_clicked();

    void on_resultsSaveButton_clicked();

    void on_resultsCleanButton_clicked();

    void on_addTireButton_clicked();

    void on_frontTireComboBox_currentIndexChanged(int index);

    void on_rearTireComboBox_currentIndexChanged(int index);

    void on_setDefaultTiresButton_clicked();

    void on_tireToPlot_toggled(bool checked);

private:
    Ui::MainWindow *ui;

    SimulationContext simCtx; 
};
#endif // MAINWINDOW_H
