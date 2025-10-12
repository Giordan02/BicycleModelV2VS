#include "src/Model/genetic_algorithm.h"


#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <random>


using namespace std;
    

// Compare individuals by fitness
bool compareFitness(const Individual& a, const Individual& b) {
    return a.fitness > b.fitness;
}

GeneticAlgorithm::GeneticAlgorithm(Vehicle vehicle,OptimizationConfig optIN, SolverConfig solIN) 
        : population(), popSize(optIN.PopSize), veh(vehicle),opt(optIN),sol(solIN), generations(opt.GenNum), minDelta(opt.minDelta), maxDelta(opt.maxDelta),
          minAlpha(opt.minAlphaf), maxAlpha(opt.maxAlphaf), minKappa(opt.minKappaf), maxKappa(opt.maxKappar), rd() {
        random_device randomDevice;
        rd.seed(randomDevice());
        progress = 0.0;         //!< Initialize with progress in 0%
        noSolution = false;     
    }

void GeneticAlgorithm::updateProgress() {
        progress += progress_step;
        int value = static_cast<int>(progress);
        emit progressChanged(value);
    }

void GeneticAlgorithm::evaluateFitness() {
        sort(population.begin(), population.end(), compareFitness);     //!< Population is ordered by it Fitness
}

double GeneticAlgorithm:: clamp(double value, double minv, double maxv) {
        return max(minv, min(maxv, value));     //!< Ensure that the value is between its bounds
    }

double GeneticAlgorithm::randomInRange(double min, double max) {
        uniform_real_distribution<> dist(min, max);     //!< Generates a random double within a given range using a uniform distribution
        return dist(rd);
    }

Individual GeneticAlgorithm:: tournamentSelection(const vector<Individual>& pop, int tournamentSize) {
    vector<Individual> tournament;

    // Randomly select individuals for the tournament
    uniform_int_distribution<> dist(0, pop.size() - 1);
    for (int i = 0; i < tournamentSize; i++) {
        int idx = dist(rd);
        tournament.push_back(pop[idx]);
    }
    // Return the winner (the one with the highest fitness)
        return *max_element(tournament.begin(), tournament.end(), compareFitness);
}    
    
void GeneticAlgorithm::crossover(const Individual& parent1, const Individual& parent2, Individual& child) {
    // Crossover for the gene
    double alpha_cross = 1.5;
    double range_delta = abs(parent1.delta - parent2.delta);
    double min_d = min(parent1.delta, parent2.delta) - range_delta * alpha_cross;
    double max_d = max(parent1.delta, parent2.delta) + range_delta * alpha_cross;
    child.delta = clamp(randomInRange(min_d, max_d), minDelta, maxDelta);

    // Uniform crossover for solver initial guess parameters
    child.alpha_F_guess = (rand() % 2 == 0) ? parent1.alpha_F_guess : parent2.alpha_F_guess;
    child.alpha_R_guess = (rand() % 2 == 0) ? parent1.alpha_R_guess : parent2.alpha_R_guess;
    child.kappa_F_guess = (rand() % 2 == 0) ? parent1.kappa_F_guess : parent2.kappa_F_guess;
    child.kappa_R_guess = (rand() % 2 == 0) ? parent1.kappa_R_guess : parent2.kappa_R_guess;

    // Inherit the best guess for velocity
    child.V_guess = max(parent1.V_guess, parent2.V_guess);
    child.Vx_guess = 0.8*child.V_guess;
    child.Vy_guess = 0.5*child.V_guess;

    // Ensure that there is no garbage value fo child fitness
    child.fitness = 0;
    child.converged = false;
}

void GeneticAlgorithm::mutate(Individual& ind) {
    double mutation_rate = 0.25;    // 25% chance to mutate each gene
    // Use normal distributions to create small changes around the current value
    normal_distribution<> dDelta(0.0, 0.01);
    normal_distribution<> dAlpha_guess(0.0, 0.05);
    normal_distribution<> dKappa_guess(0.0, 0.2);

    if (randomInRange(0.0, 1.0) < mutation_rate) {
        ind.delta = clamp(ind.delta + dDelta(rd), minDelta, maxDelta);
    }
    if (randomInRange(0.0, 1.0) < mutation_rate) {
        ind.alpha_F_guess = clamp(ind.alpha_F_guess + dAlpha_guess(rd), minAlpha, maxAlpha);
    }
    if (randomInRange(0.0, 1.0) < mutation_rate) {
        ind.alpha_R_guess = clamp(ind.alpha_R_guess + dAlpha_guess(rd), minAlpha, maxAlpha);
    }        
    if (randomInRange(0.0, 1.0) < mutation_rate) {
        ind.kappa_F_guess = clamp(ind.kappa_F_guess + dKappa_guess(rd), minKappa, maxKappa);
    }
    if (randomInRange(0.0, 1.0) < mutation_rate) {
        ind.kappa_R_guess = clamp(ind.kappa_R_guess + dKappa_guess(rd), minKappa, maxKappa);
    }

}

QString GeneticAlgorithm::generateSummary(Individual best, Vehicle veh, OptimizationConfig opt, SolverConfig sol){
    QString summary;
    // Handle the case where no solution could be found.
    if (GeneticAlgorithm::noSolution) {
        summary = "The solver failed to find solutions for the given turn radius. Please try increasing the maximum number of iterations allowed or changing the vehicle parameters.\n\n";
        return summary;
    }

    // Builds the detailed summary for Results tab in GUI
    summary += "======================\n";
    summary += "FINAL OPTIMIZED RESULT\n";
    summary += "======================\n";
    summary += QString("Max Velocity: %1 m/s\n").arg(best.fitness);
    summary += QString("Max Vx: %1 m/s\n").arg(best.Vx);
    summary += QString("Max Vy: %1 m/s\n").arg(best.Vy);
    summary += QString("Yaw Velocity: %1 degrees\n").arg(radToDegree(best.r));
    summary += QString("Max acc: %1 m/s^2\n\n").arg(best.ay);
    summary += QString("Optimized Delta: %1 degrees\n").arg(radToDegree(best.delta));
    summary += QString("Optimized Front Lateral Tire Force: %1 N\n").arg(best.MF_Fy_F);
    summary += QString("Optimized Rear Lateral Tire Force: %1 N\n").arg(best.MF_Fy_R);
    summary += QString("Optimized Front Longitudinal Tire Force: %1 N\n").arg(best.MF_Fx_F);
    summary += QString("Optimized Rear Longitudinal Tire Force: %1 N\n\n").arg(best.MF_Fx_R);
    summary += QString("Load Distribution on the front tire: %1 N\n").arg(best.Fz_F);
    summary += QString("Load Distribution on the rear tire: %1 N\n\n").arg(best.Fz_R);
    summary += QString("Front Slip Angle: %1 degree\n").arg(radToDegree(best.alpha_F));
    summary += QString("Rear Slip Angle: %1 degree\n").arg(radToDegree(best.alpha_R));
    summary += QString("Front Slip Ratio: %1 [-]\n").arg(best.kappa_F);
    summary += QString("Rear Slip Ratio: %1 [-]\n\n").arg(best.kappa_R);
    summary += "========================\n\n";

    summary += "Car Parameters:\n";
    summary += "===============\n";
    summary += QString("Turn Radius: %1 m\n").arg(veh.R);
    summary += QString("CG to Front Axle: %1 m\n").arg(veh.a);
    summary += QString("CG to Rear Axle: %1 m\n").arg(veh.b);
    summary += QString("Vehicle Mass: %1 kg\n").arg(veh.m);
    summary += "===============\n\n";

    summary += "Solver Parameters:\n";
    summary += "==================\n";
    summary += QString("Max Iterations: %1\n").arg(sol.maxIter);
    summary += "==================\n\n";

    summary += "Optimization Parameters:\n";
    summary += "========================\n";
    summary += QString("Generations: %1\n").arg(opt.GenNum);
    summary += QString("Population Size: %1\n").arg(opt.PopSize);
    summary += QString("Delta Range: [%1 , %2] degrees\n\n").arg(radToDegree(opt.minDelta)).arg(radToDegree(opt.maxDelta));
    summary += QString("Alpha_f Range: [%1 , %2] degrees\n").arg(radToDegree(opt.minAlphaf)).arg(radToDegree(opt.maxAlphar));
    summary += QString("Alpha_r Range: [%1 , %2] degrees\n").arg(radToDegree(opt.minAlphar)).arg(radToDegree(opt.maxAlphar));
    summary += QString("Kappa_f Range: [%1 , %2] [-]\n").arg(opt.minKappaf).arg(opt.maxKappar);
    summary += QString("Kappa_r Range: [%1 , %2] [-]\n").arg(opt.minKappar).arg(opt.maxKappar);
    summary += "========================\n\n";

    summary += "\nSOLVER QUALITY\n";
    summary += "===============\n";
    summary += QString("Number of Iterations: %1\n").arg(best.summary.iterations.size());
    summary += QString("Final Cost: %1\n\n").arg(best.summary.final_cost);
    summary += "Residuals:\n";
    for (int i = 0; i < 7; i++) {
        summary += QString("r[%1] = %2\n").arg(i).arg(best.residuals[i]);
    }
    summary += "===============\n\n\n";

    return summary;
}

void GeneticAlgorithm::run() {
    // --- 1. INITIALIZATION ---
    progress = 0.0;
    progress_step = 100.0 / ((generations + 1) * popSize);      // Progress step is calculate with the number of individual needed to create the population
    double Max_V_guess = 30.0;
    population.clear(); 

    // --- 2. GENERATE INITIAL POPULATION ---
    // Create the first generation of random, valid individuals.
    for (size_t i = 0; i < popSize; ++i) {
        size_t count = 0;
        bool converge = false;
        while (!converge) {
            // Loop until a converged solution is found.
            if (count > 1000) {
                // Escape hatch if the solver gets stuck.
                //std::cout << "\n\nThe solver failed to find solutions for a " << veh.R << " meters turn Radius\n\n";
                noSolution = true;
                break;
            }
            // Create a random individual.
            Individual initial;
            initial.delta = randomInRange(minDelta, maxDelta);
            initial.alpha_F_guess = randomInRange(minAlpha, maxAlpha);
            initial.alpha_R_guess = randomInRange(minAlpha, maxAlpha);
            initial.kappa_F_guess = randomInRange(minKappa, maxKappa);
            initial.kappa_R_guess = randomInRange(minKappa, maxKappa);
            initial.V_guess = Max_V_guess;
            initial.Vx_guess = randomInRange(0.0, initial.V_guess);
            initial.Vy_guess = randomInRange(0.0, 0.1 * initial.V_guess);

            // Solve for this individual's fitness.
            solveIndividual(initial, veh, sol, opt);
            if (initial.fitness != 0) {
                if (Max_V_guess < initial.fitness) {
                    Max_V_guess = initial.fitness;
                }
                population.push_back(initial);
                converge = true;
                updateProgress(); // emits progressChanged (queued to GUI thread)
            } else {
                count++;
            }
        }
        if (noSolution) break;
    }

    // If initial population failed, exit early and update progress
    if (noSolution) {
        emit summaryReady(generateSummary(Individual(), veh, opt, sol));
        emit progressChanged(100);
        emit finished();
        return;
    }


    // --- 3. GENERATIONAL LOOP ---
        for (int gen = 0; gen < generations; gen++) {
            evaluateFitness();      // Sort the current population.

            vector<Individual> newPopulation;

            // Elitism: Preserve the best individuals.
            int elite_count = popSize / 20;
            for (int i = 0; i < elite_count; i++) {
                newPopulation.push_back(population[i]);
                updateProgress();
            }

            // Mutate some of the best individuals to explore nearby solutions.
            int mutation_count = popSize / 20;
            for (int i = 0; i < mutation_count; i++) {
                Individual clone = population[i % 5];
                mutate(clone);
                solveIndividual(clone, veh, sol, opt);
                if (clone.fitness != 0) {
                    newPopulation.push_back(clone);
                    updateProgress();
                }
            }

            // Crossover: Fill the rest of the population with children.
            while (newPopulation.size() < popSize) {
                Individual parent1 = tournamentSelection(population, 3);
                Individual parent2 = tournamentSelection(population, 3);
                Individual child;
                crossover(parent1, parent2, child);
                solveIndividual(child, veh, sol, opt);
                if (child.fitness > 0) {
                    newPopulation.push_back(child);
                    updateProgress();
                }
            }
            
            population = newPopulation;
        }
        // THIS LOOP ENDS WHEN THE DESIRED NUMBER OF INDIVIDUALS IS ACHIEVED

        evaluateFitness(); // Organizes the final population
            

        /* DEBUG TOOL
        cout << "\n\n==================================================" << endl;
        cout << "               FINAL OPTIMIZED RESULT" << endl;
        cout << "==================================================" << endl;
        cout << "\nMax Velocity: " << population[0].fitness << " m/s"
             << "\nMax Vx: " << population[0].Vx << " m/s"
             << "\nMax Vy: " << population[0].Vy << " m/s"
             << "\nMax acc: " << population[0].ay << "m/s^2"
             << "\nOptimized Delta: " << population[0].delta << " degrees\n"
             << "\nBeta: " << population[0].beta << "RAD\n"
             << "\nOptimized Front Lateral Tire Force: " << population[0].MF_Fy_F << " N"
             << "\nOptimized Rear Lateral Tire Force: " << population[0].MF_Fy_R << " N"
             << "\nOptimized Front Longitudinal Tire Force: " << population[0].MF_Fx_F << " N"
             << "\nOptimized Rear Longitudinal Tire Force: " << population[0].MF_Fx_R << " N"
             << "\nLoad Distribution on the front tire: " << population[0].Fz_F << " N"
             << "\nLoad Distribution on the rear tire: " << population[0].Fz_R << " N"
             << "\nFront Slip Angle: " << population[0].alpha_F  << " degree"
             << "\nRear Slip Angle: " << population[0].alpha_R  << " degree"
             << "\nFront Slip Ratio: " << population[0].kappa_F << " [-]"
             << "\nRear Slip Ratio: " << population[0].kappa_R << " [-]";
            
             std::cout << "Residuals:\n";
            for (int i = 0; i < 7; i++) {
                std::cout << "r[" << i << "] = " << population[0].residuals[i] << "\n";
            }
        */
        
        bestIndividual = population[0];
        cout << bestIndividual.fitness << endl;

        // Generate the summary report.
        QString summary = generateSummary(population[0], veh, opt, sol);

        // Emit signals to notify the GUI that the process is complete.
        emit optimizationFinished(bestIndividual);
        emit progressChanged(100);
        emit summaryReady(summary);
        emit finished();
}