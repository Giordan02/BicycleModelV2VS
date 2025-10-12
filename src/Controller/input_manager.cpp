#include "src/Controller/input_manager.h"
#include <QThread>
#include <QObject>

double degreeToRad(double deg){
    double rad = (deg * 3.14159265359) / 180;
    return rad;
}

double radToDegree(double rad){
    double deg = (rad * 180) / 3.14159265359;
    return deg;
}

bool InputManager:: validateAndStorePosi(QLineEdit* edit, double& target){
    bool ok;
    double value = edit->text().toDouble(&ok);

    // Case 1: Input is not a valid number.
    if (!ok){
        target = -1;        // Indicate an error.
        showTooltip (edit, "Please, Use numbers and point only");
        edit->setStyleSheet("background-color: #ff4d4d; color: black;");

    // Case 2: Input is a number but not positive.
    }else if (value <= 0){
        target = -1;
        showTooltip (edit, "Please, Use a value greater than zero");
        edit->setStyleSheet("background-color: #ff4d4d; color: black;");

    // Case 3: Input is valid.
    }else{
        target = value;
        edit->setStyleSheet("background-color: #99e699; color: black;");
    }
    return ok;
}

bool InputManager:: validateAndStore(QLineEdit* edit, double& target){
    bool ok;
    double value = edit->text().toDouble(&ok);

    // Case 1: Input is not a valid number.
    if (!ok){
        target = 0;
        showTooltip (edit, "Please, Use numbers and point only");
        edit->setStyleSheet("background-color: #ff4d4d; color: black;");
        
    // Case 2: Input is valid.
    }else{
        target = value;
        edit->setStyleSheet("background-color: #99e699; color: black;");
    }
    return ok;
}

bool InputManager:: validateAndStoreInRad(QLineEdit* edit, double& target){
    bool ok;
    double value = edit->text().toDouble(&ok);
    value = degreeToRad(value);

    // Case 1: Input is not a valid number.
    if (!ok){
        target = 0;
        showTooltip (edit, "Please, Use numbers and point only");
        edit->setStyleSheet("background-color: #ff4d4d; color: black;");

    // Case 2: Input is valid
    }else{
        target = value;
        edit->setStyleSheet("background-color: #99e699; color: black;");
    }
    return ok;
}

bool InputManager::validateAndStoreInt(QLineEdit* edit, int& target){
    bool ok;
    int value = edit->text().toInt(&ok);

    // Case 1: Input is not a valid number.
    if (!ok || value <= 0){
        target = 0;
        showTooltip (edit, "Please, Use a positive integer");
        edit->setStyleSheet("background-color: #ff4d4d; color: black;");

    // Case 2: Input is valid
    }else{
        target = value;
        edit->setStyleSheet("background-color: #99e699; color: black;");
    }
    return ok;
}



GeneticAlgorithm* InputManager::startOptimization(OptimizationConfig& opt, SolverConfig& sol, Vehicle& veh, QProgressBar* progressBar, QLabel* statusLabel){
    if (!inputsVerification(veh, sol, opt)){
        return nullptr;
    }

    GeneticAlgorithm* ga = new GeneticAlgorithm(veh, opt, sol);
    
    // Create thread
    QThread* thread = new QThread();

    // Move GA to this thread
    ga->moveToThread(thread);

    // When thread starts, run GA
    QObject::connect(thread, &QThread::started, ga, &GeneticAlgorithm::run);

    // Progress -> progress bar
    QObject::connect(ga, &GeneticAlgorithm::progressChanged, progressBar, &QProgressBar::setValue);

    // Finished -> status label
    QObject::connect(ga, &GeneticAlgorithm::finished, [=]() {
        statusLabel->setText("Optimization finished!");
        thread->quit();   // stop the thread
    });

    // Cleanup when thread finishes
    QObject::connect(thread, &QThread::finished, ga, &QObject::deleteLater);
    QObject::connect(thread, &QThread::finished, thread, &QObject::deleteLater);

    // Start thread
    thread->start();

    return ga;
}

void InputManager:: showTooltip(QLineEdit* edit, const QString& message){
    // Maps the local widget coordinate to a global screen coordinate to position the tooltip correctly.
    QToolTip::showText(edit->mapToGlobal(QPoint(edit->height() + 10, 0)), message);
}


bool InputManager::inputsVerification(Vehicle& veh, SolverConfig& sol, OptimizationConfig opt){
    bool verified = false;

    // Check for non-positive physical parameters.
    if (veh.R <= 0 || veh.a <= 0 || veh.b <= 0 || veh.m <= 0 || veh.Cd <= 0 || veh.Af <= 0 || veh.f_r_F < 0){
        QMessageBox::warning(nullptr, "Input Error", "Please, check the vehicle inputs. All values must be greater than zero.");
    
    // Check for invalid optimization ranges and settings.
    }else if (opt.GenNum < 0 || opt.PopSize <= 0 || opt.minDelta > opt.maxDelta || opt.minAlphaf > opt.maxAlphaf || opt.minAlphar > opt.maxAlphar || opt.minKappaf > opt.maxKappaf || opt.minKappar > opt.maxKappar){
        QMessageBox::warning(nullptr, "Input Error", "Please, check the optimization inputs. Generation Number and Population Size must be greater than zero. Minimum values must be less than Maximum values.");
    
    // All checks passed.
    }else{
        verified = true;
    }

    return verified;
}
