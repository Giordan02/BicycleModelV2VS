#ifndef SIMULATIONINPUTS_H
#define SIMULATIONINPUTS_H

#include <cmath>
#include <QMap>
#include "src/Model/tire_model.h"
#include "ceres/ceres.h"

/*
    simulation_inputs organizes all inputs involved in the code,
    storing the into variables that belongs to specified classes
*/

using namespace std;

// Constants
const double g = 9.81;  // Gravitational acceleration
extern double rho;      // Air density defined in VehicleInfo.cpp

// The Vehicle information is separed in two cathegories:
// 1. Vehicle characteristics (struct Vehicle): fixed parameters that are used at every simulation and for all the vehicles of the model
// 2. Individual (struct Individual): parameters that define a specific vehicle and its performance in the simulation

/**
 * @struct Vehicle
 * @brief Stores the fixed physical characteristics of the vehicle model.
 * These parameters are constant throughout a simulation.
 */

struct Vehicle {
    double R;           // Turn Radius
    double a;           // CG to Front Axle
    double b;           // CG to Rear Axle
    double m;           // Vehicle mass
    double gamma_w;     // Wheels inclination angle
    double Cd;          // Drag Coefficient
    double Af;          // Frontal Area
    double f_r_F;       // Rolling Resistance coefficient

    PacejkaParams FrontTire; // Front tire parameters
    PacejkaParams RearTire;  // Rear tire parameters

    Vehicle();
    Vehicle(double r, double a_val, double b_val, double mi, double gamma, double cd, double af, double f_r);
};

// Struct for an individual

struct Individual {
    double delta;               // Wheel Steering angle
    double alpha_F_guess, alpha_R_guess, kappa_F_guess, kappa_R_guess, V_guess, Vx_guess, Vy_guess;     // Initial guess for Solver
    double fitness;             // Fitness = velocity (higher is better)
    double Vx, Vy;              // Velocity Components
    double Fz_F, Fz_R;          // Vertical load on the tires
    double MF_Fx_F, MF_Fx_R;    // Longitudinal Forces on the tires
    double MF_Fy_F, MF_Fy_R;    // Lateral Forces on the tires
    double r, beta;             // Vehicle's angular velocity and sideslip angle
    double ay;                  // Lateral acceleration
    double alpha_F, alpha_R;    // Tire slip angles
    double kappa_F, kappa_R;    // Tire slip ratios
    bool converged;             // Solver convergence status
    double Fres_f;              // Rolling resistance force on the front axle
    double F_D;                 // Aerodynamic drag force

    ceres::Solver::Summary summary; // Store the summary of the solver

    std::array<double, 7> residuals; // Store the residuals after solving

    Individual();
    Individual(double d, double guess);

    void defineGuesses(double alphaF, double alphaR, double kappaF, double kappaR, double V, double Vx, double Vy);
};

/**
 * @struct SolverConfig
 * @brief Holds configuration parameters for the numerical solver.
 */

struct SolverConfig {
    int maxIter = 100;        // Max quantity of iterations allowed for each solver call (standard value = 100)
    vector<double> Tolerances = vector<double>(7, 1e-6); // Vector of size 7, initialized to 10E6
};

/**
 * @struct OptimizationConfig
 * @brief Holds configuration parameters and bounds for the optimization algorithm.
 */

struct OptimizationConfig {
    int GenNum = 1;
    int PopSize = 1;
    double minDelta = -0.3, maxDelta = 0.3;
    double minAlphaf = - 0.27, maxAlphaf = 0.27, minAlphar = - 0.27, maxAlphar = 0.27;
    double minKappaf = -0.1, maxKappaf = 0.1, minKappar = - 0.1, maxKappar = 0.1;
    double minV = 0.0, maxV = 100.0;
    double minVx = 0.0, maxVx = 100.0;
    double minVy = -50.0, maxVy = 50.0;

};

/**
 * @struct tireInputs
 * @brief Template struct to hold input parameters for tire force plotting and calculations.
 * @tparam T The numeric type (e.g., double, ceres::Jet).
 */

template <typename T>
struct tireInputs{
    T normalForce = 50;        // Non-Zero value
    T inclinationAngle = 0;
    T kappa = 0.0;
    T alpha = 0.0;

    // --------------- Standart Pacejka Tire Parameters --------------- //

    // Creation of the Pacejka tire parameters for front and rear tires
    PacejkaParams Tire = createTireParams ("Front Tire Input Default",
                                        1.622, 1.275, -0.1237, 0,          // p_Cx1, p_Dx1, p_Dx2, p_Dx3
                                        -0.1048, 0.7129, 0.3907, 0,          // p_Ex1, p_Ex2, p_Ex3, p_Ex4
                                        23.755, 2.195, 0.3222,               // p_Kx1, p_Kx2, p_Kx3
                                        0, 0,                                // p_Hx1, p_Hx2
                                        0, 0,                                // p_Vx1, p_Vx2
                                        19.278, -14.019, 0.9819,             // r_Bx1, r_Bx2, r_Cx1
                                        0, 0,                                // r_Ex1, r_Ex2
                                        0,                                   // r_Hx1
                                        1.4872, 1.0488, -0.23, 0.8878,       // p_Cy1, p_Dy1, p_Dy2, p_Dy3
                                        -0.8996, -0.5536, 0, -5.2807,      // p_Ey1, p_Ey2, p_Ey3, p_Ey4
                                        16.859, 1.9348, 0.1695,              // p_Ky1, p_Ky2, p_Ky3
                                        0, 0, 0.0041,                        // p_Hy1, p_Hy2, p_Hy3
                                        0, 0, 0.5365, 0.4555,                // p_Vy1, p_Vy2, p_Vy3, p_Vy4
                                        6.9875, 7.2, 0, 1.0074,              // r_By1, r_By2, r_By3, r_Cy1
                                        0, 0,                                // r_Ey1, r_Ey2
                                        0, 0,                                // r_Hy1, r_Hy2
                                        0, 0, 0, 0, 0, 0,                    // r_Vy1, r_Vy2, r_Vy3, r_Vy4, r_Vy5, r_Vy6
                                        8.6458, -1.0905, -2.8235, 0, 0.4723, 9.4811, 0,    // q_Bz1, q_Bz2, q_Bz3, q_Bz4, q_Bz5, q_Bz9, q_Bz10
                                        1.1479,                                             // q_Cz1
                                        0.1232, -0.0086, 0, -0.2289, 0, 0, 0.2597, 0.0279,  // q_Dz1, q_Dz2, q_Dz3, q_Dz4, q_Dz6, q_Dz7, q_Dz8, q_Dz9
                                        -3.2802, -0.7523, 0, 0, -3.9464,                    // q_Ez1, q_Ez2, q_Ez3, q_Ez4, q_Ez5
                                        0, 0, 0.0329, 0.0465,                               // q_Hz1, q_Hz2, q_Hz3, q_Hz4
                                        0, 0.0458, -0.9372, 0.504,                          // S_Sz1, S_Sz2, S_Sz3, S_Sz4
                                        1, 1, 1.4, 1, 1, 1, 1, 1,           // lambda_gammax, lambda_Cx, lambda_mux, lambda_Ex, lambda_Kx, lambda_Hx, lambda_Vx, lambda_xalpha
                                        1.45, 2, 1, 1, 1, 1, 1, 1, 1,        // lambda_muy, lambda_Ky, lambda_gammay, lambda_Cy, lambda_Ey, lambda_Hy, lambda_Vy, lambda_Vykappa, lambda_ykappa
                                        1, 5782, 1,                         // lambda_Fz0, F_z0, lambda_S
                                        1,1,1,                              //lambda_gammaz, lambda_t, lambda_r
                                        0.3160);
};

/**
 * @struct tireResults
 * @brief Stores the calculated output forces and moments from the tire model.
 * Used exclusively to plot tire forces and aligning moment.
 */

struct tireResults{
    double slipAngle;           
    double slipRatio;
    double lateralForce;
    double longitudinalForce;
    double aligningMoment;
};

//! Sets the front and rear tire parameters to a default configuration.
void setDefaultTires(PacejkaParams &frontTire, PacejkaParams &rearTire);

/**
 * @class SimulationContext
 * @brief A container class that holds all simulation data, configurations, and results.
 * This acts as a central hub for passing simulation state between different parts of the application.
 */
class SimulationContext {
public:
    Vehicle veh;                            //!< The vehicle's physical characteristics.
    SolverConfig sol;                       //!< Solver configuration.
    OptimizationConfig opt;                 //!< Optimization algorithm configuration.
    tireInputs<double> tire;                //!< Input parameters for tire plotting.
    QMap<QString, PacejkaParams> m_tires;   //!< A map to store different named tire models.
    int runCount = 1;                       //!< A counter for the number of simulation runs.
    QString resultsText;                    //!< A string to store formatted results for display.
};


#endif // SIMULATIONINPUTS_H
