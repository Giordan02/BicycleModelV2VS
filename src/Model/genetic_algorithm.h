#ifndef GENETICALGORITHM_H
#define GENETICALGORITHM_H
#pragma once

#include "src/model/eqn_solver.h"
#include "src/controller/simulation_inputs.h"
#include "src/Model/tire_model.h"
#include "src/Controller/input_manager.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <random>
#include <QObject>

/**
 * @brief Compares two Individual structs based on their fitness.
 * Used for sorting the population in descending order (higher fitness is better).
 * @param a The first Individual.
 * @param b The second Individual.
 * @return True if Individual a has a higher fitness than Individual b.
 */

bool compareFitness(const Individual& a, const Individual& b);

/**
 * @class GeneticAlgorithm
 * @brief Implements a genetic algorithm to optimize vehicle performance.
 *
 * This class uses principles of evolution—selection, crossover, and mutation—to
 * find the optimal steering angle (delta) and the best initial guesses that can
 * find the best vehicle's cornering velocity, subject to the physical constraints of
 * the vehicle dynamics model. It inherits from QObject to emit signals for GUI updates.
 */

class GeneticAlgorithm : public QObject {
    Q_OBJECT
private:

    void updateProgress();      //!< Call to update the progress bar at the GUI
    void evaluateFitness();     //!< Sorts the population by fitness in descending order.
    Individual tournamentSelection(const std::vector<Individual>& pop, int tournamentSize);     //!< Selects a parent from the population using a tournament.
    void crossover(const Individual& parent1, const Individual& parent2, Individual& child);    //!< Creates a child by combining genes from two parents.
    void mutate(Individual& ind);       //!< Applies small, random changes to an individual's genes.

    double randomInRange(double min, double max);           //!< Generates a random double within a specified range.
    double clamp(double value, double minv, double maxv);   //!< Clamps a value between a minimum and maximum.
    QString generateSummary(Individual best, Vehicle veh, OptimizationConfig opt, SolverConfig sol);    //!< Creates a formatted summary string of the results.

    // Private member variables
    std::vector<Individual> population;     //!< The current population of solutions (vector of individuals).
    size_t popSize;                         //!< The number of individuals in the population.
    Vehicle veh;                            //!< The vehicle's fixed physical parameters.
    OptimizationConfig opt;                 //!< Configuration for the optimization process.
    SolverConfig sol;                       //!< Configuration to use in the equation solver.
    int generations;                        //!< The number of generations (later defined with opt).

    double progress_step;                   //!< Step used in progress bar
    double progress;                        //!< Stored Progrees of the current optimization
    

    // Parameter ranges
    double minDelta;
    double maxDelta;
    double minAlpha;
    double maxAlpha;
    double minKappa;
    double maxKappa;

    // Random number generator
    std::mt19937 rd;

public:
    bool noSolution = false;    //!< if it is not possible to find a solution with current configs, it returns true

    Individual bestIndividual;  //!< Best Individual of the population is stored here

    /**
     * @brief Constructor for the GeneticAlgorithm class.
     * @param vehicle The Vehicle object with fixed parameters.
     * @param optIN The OptimizationConfig with GA settings.
     * @param solIN The SolverConfig for the equation solver.
     */

    GeneticAlgorithm(Vehicle vehicle, OptimizationConfig optIN, SolverConfig solIN);

    /**
     * @brief The main entry point to start the genetic algorithm optimization.
     */
    void run();

signals:
    /**
     * @brief Emitted periodically to update a progress bar in the GUI.
     * @param value The current progress percentage (0-100).
     */
    void progressChanged(int value);  

    /**
     * @brief Emitted when the entire optimization process has finished.
     */
    void finished();                 

    /**
     * @brief Emitted when the optimization is finished, carrying the best result.
     * @param best The best Individual found by the algorithm.
     */
    void optimizationFinished(const Individual& best);
    
    /**
     * @brief Emitted when the final summary report is ready.
     * @param summary A formatted QString containing the detailed results.
     */
    void summaryReady(QString summary);
};

#endif 