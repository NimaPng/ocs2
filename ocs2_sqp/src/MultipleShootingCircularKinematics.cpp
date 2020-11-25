#include <ocs2_oc/test/circular_kinematics.h>
#include "ocs2_sqp/MultipleShootingSolver.h"

int main(int argc, char **argv)
{
    ocs2::CircularKinematicsSystem system;
    ocs2::CircularKinematicsCost cost;
    cost.initialize("circular_kinematics_cost", "/tmp/ocs2", true, false);
    ocs2::CircularKinematicsConstraints constraint;

    // Set this one up.
    ocs2::MultipleShootingSolverSettings settings;
    settings.N = 10;
    settings.nx = 2;
    settings.nu = 2;
    settings.sqpIteration = 5;
    settings.constrained = true;
    settings.printPrimalSol = false;
    settings.printSolverStatistics = true;
    settings.printSolverStatus = true;
    settings.initPrimalSol = false;
    ocs2::MultipleShootingSolver solver(settings, &system, &cost, &constraint);

    const ocs2::scalar_t startTime = 0.0;
    const ocs2::scalar_t finalTime = 1.0;
    const ocs2::vector_t initState = (ocs2::vector_t(2) << 1.0, 0.0).finished(); // radius 1.0
    const ocs2::scalar_array_t partitioningTimes{startTime, (startTime + finalTime) / 2.0, finalTime}; // doesn't matter 
    solver.run(startTime, initState, finalTime, partitioningTimes);
    solver.printPrimalSolution();

    // Successful exit
    return 0;
}