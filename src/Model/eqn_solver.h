#ifndef EQNSOLVER_H
#define EQNSOLVER_H

#include "src/controller/simulation_inputs.h"
#include <ceres/ceres.h>

using namespace ceres;

/**
 * @struct ResidualFunctor
 * @brief A Ceres cost functor that calculates the residuals for the vehicle dynamics equations.
 * This struct implements operator() which Ceres uses to evaluate the system of nonlinear equations.
 * The goal of the solver is to find a set of inputs that makes all residuals as close to zero as possible.
 */

struct ResidualFunctor {
    /**
     * @brief Constructor for the ResidualFunctor.
     * @param v A constant reference to the Vehicle's fixed parameters.
     * @param ind A constant reference to the Individual's current state (used for delta).
     */
    ResidualFunctor(const Vehicle& v,const Individual& ind) : veh_(v), ind_(ind) {}

    /**
     * @brief The core evaluation function called by Ceres Solver.
     * It takes the current estimates of the solver variables and calculates the 7 residuals.
     * @tparam T The numeric type, which will be `double` for evaluation or `ceres::Jet` for automatic differentiation.
     * @param alpha_f Pointer to the current estimate for the front slip angle.
     * @param alpha_r Pointer to the current estimate for the rear slip angle.
     * @param kappa_f Pointer to the current estimate for the front slip ratio.
     * @param kappa_r Pointer to the current estimate for the rear slip ratio.
     * @param V Pointer to the current estimate for the total velocity.
     * @param V_x Pointer to the current estimate for the longitudinal velocity.
     * @param V_y Pointer to the current estimate for the lateral velocity.
     * @param residuals Pointer to an array where the 7 calculated residual values will be stored.
     * @return true, indicating the computation was successful.
     */

    template <typename T>
    bool operator()(const T* alpha_f, const T* alpha_r, const T* kappa_f, const T* kappa_r,
                    const T* V, const T* V_x, const T* V_y, T* residuals) const {
        tireInputs<double> tire;

        // Helpers
        T r = *V / T(veh_.R);                                                   // Yaw Velocity Definition      
        T cos_delta = ceres::cos(T(ind_.delta));                                // Facilities to use cos and sin of delta with ceres
        T sin_delta = ceres::sin(T(ind_.delta));
        T F_D = T(0.5) * T(rho) * T(veh_.Cd) * T(veh_.Af) * (*V_x * *V_x);      // Aerodynamic Drag equation
        T gamma = T(veh_.gamma_w);                                              // Definition of gamma as a type T

        T Fz_f = T(veh_.m * g * veh_.b / (veh_.a + veh_.b));                    // Front Normal load calculation
        T Fz_r = T(veh_.m * g * veh_.a / (veh_.a + veh_.b));                    // Rear Normal load calculation
        
        T Fres_f = -T(veh_.f_r_F) * Fz_f;                                       // Rolling Resistance on front tire

        // Tire forces calculations with Magic Formula
        T Fx_f = calculateCombinedLongitudinalForce(veh_.FrontTire, Fz_f, *alpha_f, *kappa_f, gamma);
        T Fy_f = calculateCombinedLateralForce(veh_.FrontTire, Fz_f, *alpha_f, *kappa_f, gamma);
        T Mz_f = calculateCombinedAligningMoment(veh_.FrontTire, Fz_f, *alpha_f, *kappa_f, gamma);
        T Fx_r = calculateCombinedLongitudinalForce(veh_.RearTire, Fz_r, *alpha_r, *kappa_r, gamma);
        T Fy_r = calculateCombinedLateralForce(veh_.RearTire, Fz_r, *alpha_r, *kappa_r, gamma);
        T Mz_r = calculateCombinedAligningMoment(veh_.RearTire, Fz_r, *alpha_r, *kappa_r, gamma);

        // Scales to use on residuals equations, this aims to improve the solver quality, mantaining all residuals in the same magnitud

        T reScale1 = T (1.0 / (1000)); // Scale the residuals 0, 1 and 2 to improve numerical stability
        T reScale4 = T (1.0 / (10)); // Scale the residual 3 to improve numerical stability
        T reScale5 = T (100.0); // Scale the residuals 4 and 5 to improve numerical stability
        T reScale7 = T (1.0 / (100.0)); // Scale the residual 6 to improve numerical stability

        // Equations
        residuals[0] = (Fx_f * cos_delta - Fy_f * sin_delta + Fx_r - F_D + T(veh_.m) * (*V_y) * r) * reScale1;  // Longitudinal force balance
        residuals[1] = (Fx_f * sin_delta + Fy_f * cos_delta + Fy_r - T(veh_.m) * (*V_x) * r) * reScale1;    // Lateral force balance
        residuals[2] = (T(veh_.a) * (Fx_f * sin_delta + Fy_f * cos_delta) - T(veh_.b) * Fy_r + Mz_f + Mz_r) * reScale1;   // Moment balance
        residuals[3] = (Fx_f - Fres_f) * reScale4;  // Front longitudinal force balance at the tire -> used to find kappa_f here
        residuals[4] = (*alpha_f - (T(ind_.delta) - ceres::atan((*V_y + T(veh_.a) * r) / (*V_x + 1e-6)))) * reScale5;  // Front slip angle constraint 
        residuals[5] = (*alpha_r + ceres::atan((*V_y - T(veh_.b) * r) / (*V_x + 1e-6))) * reScale5; // Rear slip angle constraint
        residuals[6] = ((*V) * (*V) - (*V_x) * (*V_x) - (*V_y) * (*V_y)) * reScale7;    // Velocity constraint

        return true;
    }

private:
    const Vehicle& veh_;
    const Individual& ind_;
};

// Sets the upper and lower bounds for the solver's optimization variables
void setBoundaries(ceres::Problem& problem, Individual& ind, OptimizationConfig opt);

// Checks if all calculated residuals are within their specified tolerances.
bool checkResiduals(const Individual& ind, SolverConfig sol);

// Manually verifies convergence by re-calculating residuals with the final solution.
void verifyConvergence(Individual& ind, Vehicle& veh, SolverConfig sol);

// Solves the system of equations for a single Individual's state.
void solveIndividual(Individual& ind, Vehicle& veh, SolverConfig sol, OptimizationConfig opt);

// Populates the result fields of an Individual after a successful solve.
void computeIndividualResults(Individual& ind, Vehicle& veh, ceres::Solver::Summary& summary);

void testsolver();

#endif // EQNSOLVER_H
