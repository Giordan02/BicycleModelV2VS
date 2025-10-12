#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include <QString>
#include <QLineEdit>
#include <QToolTip>
#include <QOBject>
#include <QProgressBar>
#include <QLabel>
#include "src/Model/qcustomplot.h"
#include "src/controller/simulation_inputs.h"
#include "src/Model/genetic_algorithm.h"

// Forward declaration is needed because GeneticAlgorithm.h includes this file, creating a circular dependency.
class GeneticAlgorithm;

//! Converts an angle from degrees to radians.
double degreeToRad(double deg);

//! Converts an angle from radians to degrees.
double radToDegree(double rad);

/**
 * @class InputManager
 * @brief A static utility class for managing GUI input and launching the optimization process.
 *
 * This class provides methods to validate user input from QLineEdit widgets,
 * provide visual feedback, and set up and run the GeneticAlgorithm in a separate
 * thread to keep the GUI responsive.
 */
class InputManager {
public:

     /**
     * @brief Validates a QLineEdit for a positive double, stores it, and provides visual feedback.
     * @param edit Pointer to the QLineEdit widget to validate.
     * @param target Reference to the double where the valid value will be stored.
     * @return True if the input is a valid number greater than zero, false otherwise.
     */
    static bool validateAndStorePosi(QLineEdit* edit, double& target);

    /**
     * @brief Validates a QLineEdit for any double, stores it, and provides visual feedback.
     * @param edit Pointer to the QLineEdit widget to validate.
     * @param target Reference to the double where the valid value will be stored.
     * @return True if the input is a valid number, false otherwise.
     */
    static bool validateAndStore(QLineEdit* edit, double& target);

    /**
     * @brief Validates a QLineEdit for a double (in degrees), converts it to radians, stores it, and provides feedback.
     * @param edit Pointer to the QLineEdit widget to validate.
     * @param target Reference to the double where the valid value (in radians) will be stored.
     * @return True if the input is a valid number, false otherwise.
     */
    static bool validateAndStoreInRad(QLineEdit* edit, double& target);

    /**
     * @brief Validates a QLineEdit for a positive integer, stores it, and provides visual feedback.
     * @param edit Pointer to the QLineEdit widget to validate.
     * @param target Reference to the integer where the valid value will be stored.
     * @return True if the input is a valid positive integer, false otherwise.
     */
    static bool validateAndStoreInt(QLineEdit* edit, int& target);

    /**
     * @brief Starts the genetic algorithm optimization in a separate thread.
     * This function verifies all inputs, creates the GA, moves it to a new thread,
     * connects GUI signals (progress bar, status label), and starts the process.
     * @param opt The optimization configuration.
     * @param sol The solver configuration.
     * @param veh The vehicle configuration.
     * @param progressBar Pointer to the GUI progress bar to update.
     * @param statusLabel Pointer to the GUI status label to update.
     * @return A pointer to the created GeneticAlgorithm instance, or nullptr if inputs are invalid.
     */
    static GeneticAlgorithm* startOptimization(OptimizationConfig& opt, SolverConfig& sol, Vehicle& veh, QProgressBar* progressBar, QLabel* statusLabel);
    
private:
    //! A helper function to display a validation error message as a tooltip next to a QLineEdit.
    static void showTooltip(QLineEdit* edit, const QString& message);

    //! The main validation function that checks the logical consistency of all simulation parameters.
    static bool inputsVerification(Vehicle& veh, SolverConfig& sol, OptimizationConfig opt);
};

#endif // INPUTMANAGER_H

/*  -------- IMPROVEMENT FOR FUTURE WORKS ------------

- Implementing control of all functions, including the call
  for the genetic algorithm optimization

-------- ------------------------------ ------------
*/