#include "src/Model/eqn_solver.h"
#include "src/Controller/input_manager.h"
#include <iostream>

/**
 * @brief Sets the physically plausible upper and lower bounds for the solver's variables.
 * This prevents the solver from exploring unrealistic solutions.
 * @param problem The Ceres Problem object to which the bounds will be added.
 * @param ind The Individual whose guess variables will be bounded.
 * @param opt The Genetic Algorithm Config with the bouds to be configured.
 */

void setBoundaries (ceres::Problem& problem, Individual& ind, OptimizationConfig opt) {
    problem.SetParameterLowerBound(&ind.alpha_F_guess, 0, opt.minAlphaf);  
    problem.SetParameterUpperBound(&ind.alpha_F_guess, 0, opt.maxAlphaf);   
    problem.SetParameterLowerBound(&ind.alpha_R_guess, 0, opt.minAlphar);
    problem.SetParameterUpperBound(&ind.alpha_R_guess, 0, opt.maxAlphar);
    problem.SetParameterLowerBound(&ind.kappa_F_guess, 0, opt.minKappaf);
    problem.SetParameterUpperBound(&ind.kappa_F_guess, 0, opt.maxKappaf);
    problem.SetParameterLowerBound(&ind.kappa_R_guess, 0, opt.minKappar);
    problem.SetParameterUpperBound(&ind.kappa_R_guess, 0, opt.maxKappar);
    problem.SetParameterLowerBound(&ind.V_guess, 0, 0.0);           // Speed >=0
    problem.SetParameterUpperBound(&ind.V_guess, 0, 100.0);         // Max 100 m/s
    problem.SetParameterLowerBound(&ind.Vx_guess, 0, 0.0);         // Forward speed >=0
    problem.SetParameterUpperBound(&ind.Vx_guess, 0, 100.0);
    problem.SetParameterLowerBound(&ind.Vy_guess, 0, -50.0);      
    problem.SetParameterUpperBound(&ind.Vy_guess, 0, 50.0);
}


/**
 * @brief Checks if the final calculated residuals meet the required tolerance.
 * @param ind The Individual containing the final residual values.
 * @param sol The SolverConfig containing the tolerance vector.
 * @return true if all residuals are within tolerance, false otherwise.
 */

bool checkResiduals(const Individual& ind, SolverConfig sol) {
    for (size_t i = 0; i < ind.residuals.size(); ++i) {
        if (std::abs(ind.residuals[i]) > sol.Tolerances[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Manually re-evaluates the residual functor with the solver's final values.
 * This is a post-solve check to ensure the solution is physically valid and meets the convergence criteria.
 * @param ind The Individual whose convergence status needs to be verified.
 * @param veh The Vehicle parameters used in the calculation.
 * @param sol The SolverConfig containing tolerance settings.
 */

void verifyConvergence(Individual& ind, Vehicle& veh, SolverConfig sol){
    ResidualFunctor functor(veh, ind);

    // Get residuals with the functor initialization
    functor(&ind.alpha_F_guess, &ind.alpha_R_guess, &ind.kappa_F_guess, &ind.kappa_R_guess, &ind.V_guess, &ind.Vx_guess, &ind.Vy_guess, ind.residuals.data());

    // Check Residuals with solver configuration
    if (!checkResiduals(ind, sol)) {
        ind.converged = false;
        ind.fitness = 0.0;
    } else {
        ind.converged = true;
    }
}

/**
 * @brief Populates the results fields of an Individual after a successful convergence.
 * It transfers the final "guess" values to the result fields and calculates derived metrics.
 * @param ind The Individual object to populate with results.
 * @param veh The Vehicle object used for calculations.
 * @param summary The Ceres Solver summary object.
 */

void computeIndividualResults(Individual& ind, Vehicle& veh, ceres::Solver::Summary& summary) {
    if (summary.termination_type == ceres::CONVERGENCE) {
        ind.alpha_F = ind.alpha_F_guess;
        ind.alpha_R = ind.alpha_R_guess;
        ind.kappa_F = ind.kappa_F_guess;
        ind.kappa_R = ind.kappa_R_guess;
        ind.fitness = ind.V_guess;
        ind.Vx = ind.Vx_guess;
        ind.Vy = ind.Vy_guess;
        ind.ay = ind.fitness * ind.fitness / veh.R;

        // Compute additional results based on solved values
        ind.Fz_F = veh.b * veh.m * 9.81 / (veh.a + veh.b);
        ind.Fz_R = veh.a * veh.m * 9.81 / (veh.a + veh.b);
        // Longitudinal forces
        ind.MF_Fx_F = calculateCombinedLongitudinalForce(veh.FrontTire, ind.Fz_F, ind.alpha_F, ind.kappa_F, veh.gamma_w);
        ind.MF_Fy_F = calculateCombinedLateralForce(veh.FrontTire, ind.Fz_F, ind.alpha_F, ind.kappa_F, veh.gamma_w);
        ind.MF_Fx_R = calculateCombinedLongitudinalForce(veh.RearTire, ind.Fz_R, ind.alpha_R, ind.kappa_R, veh.gamma_w);
        ind.MF_Fy_R = calculateCombinedLateralForce(veh.RearTire, ind.Fz_R, ind.alpha_R, ind.kappa_R, veh.gamma_w);
        // Forces for rolling resistance and drag
        ind.Fres_f = -veh.f_r_F * ind.Fz_F;
        ind.r = (ind.fitness) / veh.R;
        ind.beta = atan2(ind.Vy, ind.Vx);
        ind.ay = (ind.fitness * ind.fitness) / veh.R;
        ind.F_D = 0.5 * rho * veh.Cd * veh.Af * ind.Vx * ind.Vx; 
        ind.summary = summary;
        ind.converged = true;
        
    } else {
        ind.converged = false;
    }
}

/**
 * @brief Configures the options for the Ceres Solver.
 * @param options A reference to the Ceres Solver::Options object to be configured.
 * @param sol The SolverConfig struct containing user-defined settings like max iterations.
 */

void configureSolver(ceres::Solver::Options& options, SolverConfig sol) {
    options.linear_solver_type = ceres::DENSE_QR;
    options.use_nonmonotonic_steps = true;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = sol.maxIter;         
    options.function_tolerance = 1e-8;       // how small the cost reduction must be
    options.gradient_tolerance = 1e-8;       // stop if gradient small enough
    options.parameter_tolerance = 1e-8;      // stop if params barely move
}

/**
 * @brief The main function to solve the vehicle dynamics for a single Individual.
 * It sets up the Ceres problem, configures the solver, runs the solve, and processes the results.
 * @param ind A reference to the Individual to be solved.
 * @param veh A reference to the Vehicle parameters.
 * @param sol A reference to the SolverConfig.
 * @param opt A referencer to the OptimizationConfig.
 */

void solveIndividual(Individual &ind, Vehicle &veh, SolverConfig sol, OptimizationConfig opt)
{
    // Set up the problem.
    ceres::Problem problem;
    ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ResidualFunctor, 7, 1, 1, 1, 1, 1, 1, 1>(new ResidualFunctor(veh, ind));

    // Create the cost function using AutoDiff, specifying 7 residuals and 7 parameter blocks of size 1.
    problem.AddResidualBlock(cost_function, loss, &ind.alpha_F_guess, &ind.alpha_R_guess, &ind.kappa_F_guess, &ind.kappa_R_guess, &ind.V_guess, &ind.Vx_guess, &ind.Vy_guess);

    // Set parameter bounds.
    setBoundaries(problem, ind, opt);
    
    // Configure the solver.
    configureSolver(options, sol);

    // Run the solver.
    Solve(options, &problem, &summary);

    // Verify convergence and compute final results.
    verifyConvergence(ind, veh, sol);

    if (ind.converged) {
        computeIndividualResults(ind, veh, summary);
    }
    
}

void testsolver(){
    Vehicle veh;
    SolverConfig sol;
    veh.R = 50.0;
    veh.a = 1.2;
    veh.b = 1.6;
    veh.m = 1600.0;
    veh.gamma_w = 0.0;
    veh.Cd = 0.32;  
    veh.Af = 1.0;
    veh.f_r_F = 0.001;

    setDefaultTires(veh.FrontTire, veh.RearTire);

    for (double i = 0.03; i <= 0.1; i += 0.001) {
        Individual ind(i, 0.1);

        ind.defineGuesses(0.0, 0.0, 0.00, 0.0, 10.0, 10.0, 0.0);
        solveIndividual(ind, veh, sol, OptimizationConfig());
        std::cout << "Guesses 2: \n" ;
        if (ind.converged) {
            std::cout << "Solver converged for delta = " << radToDegree(i) << " with fitness = " << ind.fitness << " m/s and Vy" << ind.Vy << std::endl;
            std::cout << "Residuals: ";
            for (const auto& res : ind.residuals) { std::cout << res << " "; }
            std::cout << "Number of Iteratios" << ind.summary.iterations.size()<< std::endl;
        } else {
            std::cout << "Solver did not converge for delta = " << radToDegree(i) << std::endl;
        }
    }

/*
    for (double i = 0.04; i <= 0.06; i += 0.0005) {
        Individual ind(i, 0.1);
        ind.defineGuesses(0.0, 0.0, 0.00, 0.0, 10.0, 10.0, 0.0);
        solveIndividual(ind, veh, sol, OptimizationConfig());
        std::cout << "Tangent Velocity: \n" ;
        if (ind.converged && ind.Vy < 0.1 && ind.Vy > -0.1) {
            std::cout << "Solver converged for delta = " << radToDegree(i) << " with fitness = " << ind.fitness << " m/s and Vy = " << ind.Vy << std::endl;
            std::cout << "Residuals: ";
            for (const auto& res : ind.residuals) { std::cout << res << " "; }
            std::cout << "Number of Iteratios" << ind.summary.iterations.size()<< std::endl;
        }
    }
*/

}