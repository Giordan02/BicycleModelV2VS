/**
 * @class MainWindow
 * @brief Main GUI controller for the vehicle dynamics optimization tool.
 *
 * This class provides the user interface for configuring vehicle parameters,
 * solver options, and optimization settings. It integrates with:
 * - Vehicle (vehicle parameters and tires)
 * - SolverConfig (solver tolerances and iterations)
 * - OptimizationConfig (genetic algorithm settings)
 * - Tire plots and parameter editing
 * 
 * Responsibilities:
 * - Initialize UI and default values
 * - Handle user input validation
 * - Informates the Input Maneger to run optimization
 * - Display results and manage saving
 */


#include <QPixmap>
#include "main_window.h"
#include "ui_mainwindow.h"
#include "src/Controller/input_manager.h"
#include "src/controller/simulation_inputs.h"
#include "src/Controller/plot_tire_forces.h"
#include "src/model/eqn_solver.h"
#include "src/Model/genetic_algorithm.h"
#include "src/Controller/tire_params_editor_dialog.h"
#include "src/Model/tire_model.h"


void MainWindow::adjustToScreenSize(){
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->availableGeometry();
    
    int screenWidth = screenGeometry.width();
    int screenHeight = screenGeometry.height();
    
    // Calculate appropriate window size (e.g., 80% of screen)
    int windowWidth = qMin(1047, static_cast<int>(screenWidth * 0.8));
    int windowHeight = qMin(829, static_cast<int>(screenHeight * 0.8));
    
    this->resize(windowWidth, windowHeight);
    
    // Center the window
    this->move((screenWidth - windowWidth) / 2, 
               (screenHeight - windowHeight) / 2);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{


    ui->setupUi(this);
    adjustToScreenSize();  // Adjust to screen size
    setWindowTitle("Bicycle Model V2");

    // Setup of Images used in the interdface
    setWindowIcon(QIcon(":/resources/optimumg.png"));
    QPixmap BicycleModelImage(":/resources/BicycleModel.png");
    ui->imageLabel->setPixmap(BicycleModelImage.scaled(606, 383, Qt::KeepAspectRatio));
    QPixmap EquationsImage(":/resources/Equations.png");
    QPixmap scaled = EquationsImage.scaled(ui->equationsImage->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ui->equationsImage->setPixmap(scaled);   


    // Creation and Connection of Tires utilized to plot
    this->defaultTireDatabase();
    connect(ui->frontTireRadioButton, &QRadioButton::toggled, this, &MainWindow::on_tireToPlot_toggled);
    connect(ui->rearTireRadioButton, &QRadioButton::toggled, this, &MainWindow::on_tireToPlot_toggled);
}


MainWindow::~MainWindow()
{
    delete ui;
}

//          SETTING DEFAULT VALUES ON TEXT BOXES

/**
 * @brief Sets default values into UI text boxes for solver and optimization parameters.
 * 
 * Uses the values stored in @ref sol (SolverConfig) and @ref opt (OptimizationConfig)
 * and displays them in the corresponding input fields.
 */
void MainWindow::setDefaultValues(){
    ui->maxIterationInput->setText(QString::number((simCtx.sol).maxIter));
    ui->Eqn1TolInput->setText(QString::number(simCtx.sol.Tolerances[0], 'E', 0));
    ui->Eqn2TolInput->setText(QString::number(simCtx.sol.Tolerances[1], 'E', 0));
    ui->Eqn3TolInput->setText(QString::number(simCtx.sol.Tolerances[2], 'E', 0));
    ui->Eqn4TolInput->setText(QString::number(simCtx.sol.Tolerances[3], 'E', 0));
    ui->Eqn5TolInput->setText(QString::number(simCtx.sol.Tolerances[4], 'E', 0));
    ui->Eqn6TolInput->setText(QString::number(simCtx.sol.Tolerances[5], 'E', 0));
    ui->Eqn7TolInput->setText(QString::number(simCtx.sol.Tolerances[6], 'E', 0));
    ui->genNumInput->setText(QString::number(simCtx.opt.GenNum));
    ui->PopSizeInput->setText(QString::number(simCtx.opt.PopSize));
    ui->minDeltaInput->setText(QString::number(std::round(radToDegree(simCtx.opt.minDelta))));
    ui->maxDeltaInput->setText(QString::number(std::round(radToDegree(simCtx.opt.maxDelta))));
    ui->minAlphafInput->setText(QString::number(std::round(radToDegree(simCtx.opt.minAlphaf))));
    ui->maxAlphafInput->setText(QString::number(std::round(radToDegree(simCtx.opt.maxAlphaf))));
    ui->minAlpharInput->setText(QString::number(std::round(radToDegree(simCtx.opt.minAlphar))));
    ui->maxAlpharInput->setText(QString::number(std::round(radToDegree(simCtx.opt.maxAlphar))));
    ui->minKappafInput->setText(QString::number(simCtx.opt.minKappaf));
    ui->maxKappafInput->setText(QString::number(simCtx.opt.maxKappaf));
    ui->minKapparInput->setText(QString::number(simCtx.opt.minKappar));    
    ui->maxKapparInput->setText(QString::number(simCtx.opt.maxKappar));

}

//          SETTING TIRE TAB
/**
 * @brief Configures the Tire tab UI elements.
 * 
 * Initializes slider ranges, labels, and default configurations
 * for tire forces and moment plots.
 */
void MainWindow::setTireTab(){

    // Slider and buttons preset

    ui->normalLoadSlider->setMinimum(50);
    ui->normalLoadSlider->setMaximum(12000);
    ui->minValueNormalLoadLabel->setText(QString::number(ui->normalLoadSlider->minimum()));
    ui->maxValueNormalLoadLabel->setText(QString::number(ui->normalLoadSlider->maximum()));
    ui->tireNormalLoadValeuLabel->setText(QString::number(ui->normalLoadSlider->value()));
    ui->normalLoadSlider->setSingleStep(50);
    ui->normalLoadSlider->setPageStep(100);
    ui->normalLoadSlider->setValue(6000);

    ui->IASlider->setMinimum(-50);
    ui->IASlider->setMaximum(50);
    ui->minTireIALabel->setText(QString::number((ui->IASlider->minimum())/10));
    ui->maxTireIALabel->setText(QString::number((ui->IASlider->maximum())/10));
    ui->tireIALabel->setText(QString::number((ui->IASlider->value())/10));
    ui->IASlider->setSingleStep(1);
    ui->IASlider->setPageStep(10);

    // Tire Plots Default Configuration

    ui->tireLongForce->xAxis->setLabel("Slip Ratio [-]");
    ui->tireLongForce->yAxis->setLabel("Longitudinal Force - Fx [N]");
    ui->tireLongForce->xAxis->setRange(-15, 15);
    ui->tireLongForce->yAxis->setRange(-5000, 5000);

    ui->tireLatForce->xAxis->setLabel("Slip Angle [deg]");
    ui->tireLatForce->yAxis->setLabel("Lateral Force - Fy [N]");
    ui->tireLatForce->xAxis->setRange(-0.5, 0.5);
    ui->tireLatForce->yAxis->setRange(-5000, 5000);

    ui->tireMoment->xAxis->setLabel("Slip Angle [deg]");
    ui->tireMoment->yAxis->setLabel("Aligning - Mz [Nm]");
    ui->tireMoment->xAxis->setRange(-15, 15);
    ui->tireMoment->yAxis->setRange(-50, 50);


}

    // Generation of Tire Plots
/**
 * @brief Slot triggered when "Plot Tire Forces" button is clicked.
 * 
 * Generates tire force and moment plots using the current @ref tire configuration.
 */
void MainWindow::on_plotTireForcesButton_clicked(){
    plotLongTireForce<double>(ui->tireLongForce, simCtx.tire);
    plotLatTireForce<double>(ui->tireLatForce, simCtx.tire);
    plotAlingnMoment<double>(ui->tireMoment, simCtx.tire);
}

    // Receiving Inputs for Plotting
/**
 * @brief Slot triggered when selecting which tire to plot (front or rear).
 * 
 * Updates @ref tire.Tire according to the selected radio button.
 * 
 * @param checked Boolean indicating if the radio button is active.
 */
void MainWindow::on_tireToPlot_toggled(bool checked){
    if (!checked) return;
    if (ui->frontTireRadioButton->isChecked()) {
        simCtx.tire.Tire = simCtx.veh.FrontTire;
    } else if (ui->rearTireRadioButton->isChecked()) {
        simCtx.tire.Tire = simCtx.veh.RearTire;
    }
}

/**
 * @brief Slot triggered when Inclination Angle (IA) slider is changed.
 * 
 * Updates @ref tire.inclinationAngle and displays the new value in the UI.
 * 
 * @param value Integer value from the slider (scaled by 0.1 to degrees).
 */
void MainWindow::on_IASlider_valueChanged(int value)
{
    double newValue = value / 10.0;                     // As Slider returns an integer, It is converted to a double 10 times smaller
    simCtx.tire.inclinationAngle = degreeToRad(newValue);
    ui->tireIALabel->setText(QString::number(newValue));    // Display the IA
}

/**
 * @brief Slot triggered when Normal Load slider is changed.
 * 
 * Updates @ref tire.normalForce and displays the new value in the UI.
 * 
 * @param value Integer value from the slider representing the load [N].
 */
void MainWindow::on_normalLoadSlider_valueChanged(int value){
    ui->tireNormalLoadValeuLabel->setText(QString::number(value));  // Display the normal load 
    simCtx.tire.normalForce = value * 1.0;  //Conversion to double
}

    // Creation of Add Tire Dialog


/**
 * @brief Slot triggered when "Add Tire" button is clicked.
 * 
 * Opens the TireParamsEditorDialog for editing tire parameters,
 * stores the result in @ref m_tires, updates comboboxes, and optionally
 * saves the tire data to a JSON file.
 */
void MainWindow::on_addTireButton_clicked(){
    PacejkaParams params;
    if (TireParamsEditorDialog::editParams(this, params)) {
        bool ok;
        QString tireName = params.name;

        // Store in memory
        simCtx.m_tires.insert(tireName, params);

        // Add to combobox
        ui->frontTireComboBox->addItem(tireName);
        ui->rearTireComboBox->addItem(tireName);

        // Optionally save to JSON
        QString fileName = QFileDialog::getSaveFileName(this, "Save tire as...", QDir::homePath(), "JSON (*.json)");
        if (!fileName.isEmpty()) {
            TireParamsEditorDialog dlg(params, this);
            dlg.saveToJsonFile(fileName);
        }
    }
}

/**
 * @brief Slot triggered when front tire selection changes in the combobox.
 * 
 * Updates the vehicle's @ref veh.FrontTire with the selected tire parameters.
 * 
 * @param index Index of the selected item in the combobox.
 */
void MainWindow::on_frontTireComboBox_currentIndexChanged(int index){
    if (index < 0) return;

    QString tireName = ui->frontTireComboBox->itemText(index);
    if (simCtx.m_tires.contains(tireName)) {
        PacejkaParams params = simCtx.m_tires[tireName];
        simCtx.veh.FrontTire = params;
    }
}

/**
 * @brief Slot triggered when rear tire selection changes in the combobox.
 * 
 * Updates the vehicle's @ref veh.RearTire with the selected tire parameters.
 * 
 * @param index Index of the selected item in the combobox.
 */
void MainWindow::on_rearTireComboBox_currentIndexChanged(int index){
    if (index < 0) return;

    QString tireName = ui->frontTireComboBox->itemText(index);
    if (simCtx.m_tires.contains(tireName)) {
        PacejkaParams params = simCtx.m_tires[tireName];
        simCtx.veh.RearTire = params;
    }
}

/**
 * @brief Initializes the default tire database with front and rear tires.
 * 
 * Clears the tire map, inserts default tires, and populates the tire comboboxes.
 */
void MainWindow::defaultTireDatabase(){
    PacejkaParams FTire;
    PacejkaParams RTire;
    setDefaultTires(FTire, RTire);
    simCtx.m_tires.clear();
    simCtx.m_tires.insert(FTire.name, FTire);
    simCtx.m_tires.insert(RTire.name, RTire);
    ui->frontTireComboBox->clear();
    ui->rearTireComboBox->clear();
    ui->frontTireComboBox->addItems(simCtx.m_tires.keys());
    ui->rearTireComboBox->addItems(simCtx.m_tires.keys());
    ui->frontTireComboBox->setCurrentIndex(0);
    ui->rearTireComboBox->setCurrentIndex(1);
}

/**
 * @brief Slot triggered when "Set Default Tires" button is clicked.
 * 
 * Resets the tire database to default values.
 */
void MainWindow::on_setDefaultTiresButton_clicked(){
    this->defaultTireDatabase();
}

//          VEHICLE PARAMETERS TAB
// Actions that are triggered for each button 

void MainWindow::on_radiusInput_editingFinished(){InputManager::validateAndStorePosi(ui->radiusInput, simCtx.veh.R);}

void MainWindow::on_massInput_editingFinished(){ InputManager::validateAndStorePosi(ui->massInput, simCtx.veh.m);}

void MainWindow::on_aInput_editingFinished(){ InputManager::validateAndStorePosi(ui->aInput, simCtx.veh.a);}

void MainWindow::on_bInput_editingFinished(){ InputManager::validateAndStorePosi(ui->bInput, simCtx.veh.b);}

void MainWindow::on_CdInput_editingFinished(){ InputManager::validateAndStorePosi(ui->CdInput, simCtx.veh.Cd);}

void MainWindow::on_frontalAreaInput_editingFinished(){ InputManager::validateAndStorePosi(ui->frontalAreaInput, simCtx.veh.Af);}


//          SOLVER SETUP TAB
// Actions that are triggered for each button 

void MainWindow::on_maxIterationInput_editingFinished(){ InputManager::validateAndStoreInt(ui->maxIterationInput, simCtx.sol.maxIter);}

void MainWindow::on_Eqn1TolInput_editingFinished(){ InputManager::validateAndStorePosi(ui->Eqn1TolInput, simCtx.sol.Tolerances[0]);}

void MainWindow::on_Eqn2TolInput_editingFinished(){ InputManager::validateAndStorePosi(ui->Eqn2TolInput, simCtx.sol.Tolerances[1]);}

void MainWindow::on_Eqn3TolInput_editingFinished(){ InputManager::validateAndStorePosi(ui->Eqn3TolInput, simCtx.sol.Tolerances[2]);}

void MainWindow::on_Eqn4TolInput_editingFinished(){ InputManager::validateAndStorePosi(ui->Eqn4TolInput, simCtx.sol.Tolerances[3]);}

void MainWindow::on_Eqn5TolInput_editingFinished(){ InputManager::validateAndStorePosi(ui->Eqn5TolInput, simCtx.sol.Tolerances[4]);}

void MainWindow::on_Eqn6TolInput_editingFinished(){ InputManager::validateAndStorePosi(ui->Eqn6TolInput, simCtx.sol.Tolerances[5]);}

void MainWindow::on_Eqn7TolInput_editingFinished(){ InputManager::validateAndStorePosi(ui->Eqn7TolInput, simCtx.sol.Tolerances[6]);}

//          OPTIMIZATION TAB
// Actions that are triggered for each button 

void MainWindow::on_genNumInput_editingFinished(){ InputManager::validateAndStoreInt(ui->genNumInput, simCtx.opt.GenNum);}

void MainWindow::on_PopSizeInput_editingFinished(){ InputManager::validateAndStoreInt(ui->PopSizeInput, simCtx.opt.PopSize);}

void MainWindow::on_minDeltaInput_editingFinished(){ InputManager::validateAndStoreInRad(ui->minDeltaInput, simCtx.opt.minDelta);}

void MainWindow::on_maxDeltaInput_editingFinished(){ InputManager::validateAndStoreInRad(ui->maxDeltaInput, simCtx.opt.maxDelta);}

void MainWindow::on_minAlphafInput_editingFinished(){ InputManager::validateAndStoreInRad(ui->minAlphafInput, simCtx.opt.minAlphaf);}

void MainWindow::on_maxAlphafInput_editingFinished(){ InputManager::validateAndStoreInRad(ui->maxAlphafInput, simCtx.opt.maxAlphaf);}

void MainWindow::on_minAlpharInput_editingFinished(){ InputManager::validateAndStoreInRad(ui->minAlpharInput, simCtx.opt.minAlphar);}

void MainWindow::on_maxAlpharInput_editingFinished(){ InputManager::validateAndStoreInRad(ui->maxAlpharInput,simCtx.opt.maxAlphar);}

void MainWindow::on_minKappafInput_editingFinished(){ InputManager::validateAndStore(ui->minKappafInput, simCtx.opt.minKappaf);}

void MainWindow::on_maxKappafInput_editingFinished(){ InputManager::validateAndStore(ui->maxKappafInput, simCtx.opt.maxKappaf);}

void MainWindow::on_minKapparInput_editingFinished(){ InputManager::validateAndStore(ui->minKapparInput, simCtx.opt.minKappar);}

void MainWindow::on_maxKapparInput_editingFinished(){ InputManager::validateAndStore(ui->maxKapparInput, simCtx.opt.maxKappar);}


/**
 * @brief Slot triggered when the "Calculate" button is clicked.
 * 
 * Send information needed to start the genetic algorithm optimization
 * process using the current solver, vehicle, and optimization configurations. 
 * 
 * - Updates the progress bar and status label during execution.
 * - Connects optimization signals to update results text, plots, and UI labels.
 * - Displays the best individual's parameters (velocity, acceleration, slip angles, forces, etc.).
 */
void MainWindow::on_calculateButton_clicked()
{
    // Update Status Label
    ui->resultsStatusLabel->setText("Optimization running...");
    ui->resultsProgressBar->setValue(0);

    // Ask InputManager to run the GA and store it in ga
    GeneticAlgorithm* ga = InputManager::startOptimization (simCtx.opt, simCtx.sol, simCtx.veh, ui->resultsProgressBar, ui->resultsStatusLabel);
    
    if (ga){
        // Conecting Results Text Box generated by the GA to the Results Tab 
        QObject::connect(ga, &GeneticAlgorithm::summaryReady, this, [=](const QString& summary){ 
            simCtx.resultsText += QString("=======OPTIMIZATION RUN %1 ===========\n\n").arg(simCtx.runCount);
            simCtx.resultsText += summary + "\n";
            ui->resultsTextEdit->setPlainText(simCtx.resultsText);
            simCtx.runCount++;
        });
        
        // Conecting Results Text Label to GA Results

        QObject::connect(ga, &GeneticAlgorithm::optimizationFinished, this, [=](const Individual& best){ 

        ui->velocityLabel->setText(QString::number(best.fitness));
        ui->accelerationLabel->setText(QString::number(best.ay));
        ui->steerLabel->setText(QString::number(radToDegree(best.delta)));
        ui->CGSlipAngleLabel->setText(QString::number(radToDegree(best.beta)));
        ui->yawVelocityLabel->setText(QString::number(radToDegree(best.r)));
        ui->Fz_FLabel->setText(QString::number(best.Fz_F));
        ui->Fz_RLabel->setText(QString::number(best.Fz_R));
        ui->MF_Fx_FLabel->setText(QString::number(best.MF_Fx_F));
        ui->MF_Fx_RLabel->setText(QString::number(best.MF_Fx_R));
        ui->MF_Fy_FLabel->setText(QString::number(best.MF_Fy_F));
        ui->MF_Fy_RLabel->setText(QString::number(best.MF_Fy_R));
        ui->alpha_FLabel->setText(QString::number(radToDegree(best.alpha_F)));
        ui->alpha_RLabel->setText(QString::number(radToDegree(best.alpha_R)));
        ui->kappa_FLabel->setText(QString::number(best.kappa_F));
        ui->kappa_RLabel->setText(QString::number(best.kappa_R));
        });
    }
}

//          RESULTS TAB

/**
 * @brief Slot triggered when the "Save Results" button is clicked.
 * 
 * Opens a file dialog to select a path and saves the optimization results
 * (contents of the results text edit) into a text file.
 * 
 * If the file cannot be opened, displays a warning message box.
 */
void MainWindow::on_resultsSaveButton_clicked(){
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Results"), QDir::homePath(), tr("Text Files (*.txt);;All Files (*)"));

    if (fileName.isEmpty()) {
        return; // user canceled
    }

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("Save Error"), tr("Cannot open file %1:\n%2.").arg(QDir::toNativeSeparators(fileName),file.errorString()));
        return;
    }

    QTextStream out(&file);
    out << ui->resultsTextEdit->toPlainText();
    file.close();
}

/**
 * @brief Slot triggered when the "Save Results" button is clicked.
 * 
 * Opens a file dialog to select a path and saves the optimization results
 * (contents of the results text edit) into a text file.
 * 
 * If the file cannot be opened, displays a warning message box.
 */
void MainWindow::on_resultsCleanButton_clicked(){
    simCtx.resultsText = "";
    ui->resultsTextEdit->setPlainText(simCtx.resultsText);
    simCtx.runCount = 1;
}

/*  -------- IMPROVEMENT FOR FUTURE WORKS ------------

- Letting on_calculateButton_clicked just for calling the Input Manager for calculation, letting it handle
  the call for the Optimization Function and the display of the results (This function was builded thinking
  about the integration of the Loading Bar, made in a manner that do not stops when the optimization is 
  running, but as a result ended up bringing work to the mainwindow - that is, a task that should be attributed
  to the Controller, and not the Viwer in a MVC architeture).

- The same happens at the tire tab, where all functions could be passed to a tire Manager for example, the actual
  Scructure was build to not overcomplicate the connection of the main window interface passing to others files, a
  way of facilitating it, is implementing classes exclusivelly for the elements of each tab and passing it to the
  Input Manager at the MainWindow.

*/