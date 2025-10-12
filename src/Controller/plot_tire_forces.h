#ifndef PLOTTIREFORCES_H
#define PLOTTIREFORCES_H

#include "src/Model/qcustomplot.h"
#include "src/controller/simulation_inputs.h"
#include "src/Model/tire_model.h"
#include "src/Controller/input_manager.h"

/*
    plot_tire_forces controlls and configure the plotting of the tire graphs 
    to the use interface.
*/


/**
 * @brief Configures and displays a tire force plot on a QCustomPlot widget.
 *
 * This function initializes the graph with provided X and Y data vectors, 
 * sets axis labels and ranges, and automatically adjusts the Y-axis range. 
 * It also enables user interactions such as zooming, dragging, selecting
 * plot elements and connects a mouse-move event to show the cursor's X and
 * Y coordinates as a label on the plot.
 *
 * @param tireplot Pointer to the QCustomPlot widget where the graph will be displayed.
 * @param x QVector containing the X-axis data points.
 * @param y QVector containing the Y-axis data points.
 * @param xLabel Label text for the X-axis.
 * @param yLabel Label text for the Y-axis.
 */
void configure_plot(QCustomPlot *tireplot, QVector<double> x, QVector<double> y, QString xLabel, QString yLabel);


/**
 * @file plotTireForces.h
 * @brief Functions for plotting tire force and moment curves using QCustomPlot.
 * 
 * Provides visualization utilities to generate longitudinal, lateral,
 * and aligning moment plots.
 */


 /**
 * @brief Plots the pure longitudinal tire force as a function of slip ratio.
 * 
 * This function simulates tire longitudinal forces over a defined range
 * of slip ratios (-0.5 to 0.5) using the specified tire model and normal load,
 * and plots the resulting curve on a QCustomPlot widget.
 * 
 * @tparam T Numeric type (e.g., double or ceres::Jet<T,N>).
 * @param tireplot Pointer to the QCustomPlot widget where the graph will be drawn.
 * @param tire Structure containing tire parameters (normal load, inclination angle, etc.).
 */
template <typename T>
void plotLongTireForce(QCustomPlot *tireplot, tireInputs<T> tire) {
    std::vector<tireResults> results;

    // Creation of x and y data of the plot
    for (double kappa = -0.5; kappa <= 0.5; kappa += 0.005) {
        tireResults r;
        r.slipRatio = kappa;
        T Fz = T(tire.normalForce);
        T k = T (kappa);
        T IA = T (tire.inclinationAngle);

        r.longitudinalForce = calculatePureLongitudinalForce(tire.Tire, Fz, k, IA);

        results.push_back(r);
    }

    // Computing data into a vector
    QVector<double> x, y;
    for (const auto& r : results) {
        x.push_back(r.slipRatio);
        y.push_back(r.longitudinalForce);
    }

    // Configuring plot
    configure_plot(tireplot, x, y, "Slip Ratio [-]", "Longitudinal Force [N]");
};

/**
 * @brief Plots the pure lateral tire force as a function of slip angle.
 * 
 * This function calculates tire lateral forces over a range of slip angles
 * (-15° to 15°) and plots them on a QCustomPlot widget.
 * 
 * @tparam T Numeric type (e.g., double or ceres::Jet<T,N>).
 * @param tireplot Pointer to the QCustomPlot widget where the graph will be drawn.
 * @param tire Structure containing tire parameters (normal load, inclination angle, etc.).
 */
template <typename T>
void plotLatTireForce(QCustomPlot *tireplot, tireInputs<T> tire) {
    std::vector<tireResults> results;

    // Creation of x and y data of the plot
    for (auto alpha = -15.0; alpha <= 15.0; alpha += 0.01) {
        tireResults r;
        r.slipAngle = alpha;
        T Fz = T(tire.normalForce);
        T a = T (degreeToRad(alpha));
        T IA = T (tire.inclinationAngle);

        r.lateralForce = calculatePureLateralForce(tire.Tire, Fz, a, IA);
        results.push_back(r);
    }

    // Computing data into a vector
    QVector<double> x, y;
    for (const auto& r : results) {
        x.push_back(r.slipAngle);
        y.push_back(r.lateralForce);
    }

    // Configuring plot
    configure_plot(tireplot, x, y, "Slip Angle [deg]", "Lateral Force [N]");
}


/**
 * @brief Plots the pure aligning moment as a function of slip angle.
 * 
 * This function calculates the aligning moment for slip angles ranging
 * from -15° to 15° and displays the resulting curve.
 * 
 * @tparam T Numeric type (e.g., double or ceres::Jet<T,N>).
 * @param tireplot Pointer to the QCustomPlot widget where the graph will be drawn.
 * @param tire Structure containing tire parameters (normal load, inclination angle, etc.).
 */
template <typename T>
void plotAlingnMoment(QCustomPlot *tireplot, tireInputs<T> tire) {
    std::vector<tireResults> results;

    // Creation of x and y data of the plot
    for (auto alpha = -15.0; alpha <= 15.0; alpha += 0.01) {
        tireResults r;
        r.slipAngle = alpha;
        T Fz = T(tire.normalForce);
        T a = T (degreeToRad(alpha));
        T IA = T (tire.inclinationAngle);

        r.aligningMoment = calculatePureAligningMoment(tire.Tire, Fz, a, IA);
        results.push_back(r);
    }

    // Computing data into a vector
    QVector<double> x, y;
    for (const auto& r : results) {
        x.push_back(r.slipAngle);
        y.push_back(r.aligningMoment);
    }

    // Configuring plot
    configure_plot(tireplot, x, y, "Slip Angle [deg]", "Aligning Moment [Nm]");
}

#endif // PLOTTIREFORCES_H

/*  -------- IMPROVEMENT FOR FUTURE WORKS ------------

- Separation of the creation of the x and y values to specific function in 
  the Model, returning only tire results, and letting plot_tire_forces 
  responsible only for plotting, making it part of View instad of Controller.

- Implement the template definitions o .tpp file, keeping the header clean to
  documentation.

-------- ------------------------------ ------------
*/