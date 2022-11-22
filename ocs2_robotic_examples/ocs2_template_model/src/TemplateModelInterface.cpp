/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <string>

#include "ocs2_template_model/TemplateModelInterface.h"
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
    namespace template_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
        TemplateModelInterface::TemplateModelInterface(const std::string &taskFile, const std::string &libraryFolder,
                                                       bool verbose) {
            // check that task file exists
            boost::filesystem::path taskFilePath(taskFile);
            if (boost::filesystem::exists(taskFilePath)) {
                std::cerr << "[TemplateModelInterface] Loading task file: " << taskFilePath << std::endl;
            } else {
                throw std::invalid_argument(
                        "[TemplateModelInterface] Task file not found: " + taskFilePath.string());
            }
            // create library folder if it does not exist
            boost::filesystem::path libraryFolderPath(libraryFolder);
            boost::filesystem::create_directories(libraryFolderPath);
            std::cerr << "[TemplateModelInterface] Generated library path: " << libraryFolderPath << std::endl;

            // Default initial condition and final goal
            loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
            loadData::loadEigenMatrix(taskFile, "finalGoal", finalGoal_);

            // DDP-MPC settings
            ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
            mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
            sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting");

            /*
             * ReferenceManager & SolverSynchronizedModule
             */
            referenceManagerPtr_.reset(new SwitchedModelReferenceManager());

            /*
             * Optimal control problem
             */
            // Cost
            matrix_t Q(STATE_DIM, STATE_DIM);
            matrix_t R(INPUT_DIM, INPUT_DIM);
            matrix_t Qf(STATE_DIM, STATE_DIM);
            loadData::loadEigenMatrix(taskFile, "Q", Q);
            loadData::loadEigenMatrix(taskFile, "R", R);
            loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
            std::cerr << "Q:  \n" << Q << "\n";
            std::cerr << "R:  \n" << R << "\n";
            std::cerr << "Q_final:\n" << Qf << "\n";

            problem_.costPtr->add("cost", std::unique_ptr<StateInputCost>(new QuadraticStateInputCost(Q, R)));
            problem_.finalCostPtr->add("finalCost", std::unique_ptr<StateCost>(new QuadraticStateCost(Qf)));
            problem_.equalityConstraintPtr->add("switch", std::unique_ptr<InputEqualityConstraints>(
                    new InputEqualityConstraints(referenceManagerPtr_.get())));


            RelaxedBarrierPenalty::Config barrierPenaltyConfig;
            barrierPenaltyConfig.mu = 10;
            std::unique_ptr<InputInequalityConstriants> inputLimitPtr(new InputInequalityConstriants(referenceManagerPtr_.get()));
            std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));
            problem_.softConstraintPtr->add("inputLimit", std::unique_ptr<StateInputCost>(
                    new StateInputSoftConstraint(std::move(inputLimitPtr), std::move(penalty))));

            RelaxedBarrierPenalty::Config barrierPenaltyConfig_state;
            barrierPenaltyConfig_state.mu = 10;
            std::unique_ptr<StateConstraints> stateConstraints(new StateConstraints(referenceManagerPtr_.get()));
            std::unique_ptr<PenaltyBase> penalty_state(new RelaxedBarrierPenalty(barrierPenaltyConfig_state));
            problem_.softConstraintPtr->add("stateConstraints", std::unique_ptr<StateInputCost>(
                    new StateInputSoftConstraint(std::move(stateConstraints), std::move(penalty_state))));

            // Dynamics
            problem_.dynamicsPtr.reset(new TemplateModelDynamics(libraryFolder, verbose));

            // Rollout
            auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", verbose);
            rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

            // Initialization
            initializerPtr_.reset(new MyInitializer());
        }

    }  // namespace double_integrator
}  // namespace ocs2
