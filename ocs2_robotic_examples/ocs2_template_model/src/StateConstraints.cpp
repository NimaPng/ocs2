//
// Created by czf on 11/2/22.
//

#include "ocs2_template_model/StateConstraints.h"

namespace ocs2 {
    namespace template_model {
        StateConstraints::StateConstraints(const SwitchedModelReferenceManager *referenceManagerPtr) :
                StateInputConstraint(ConstraintOrder::Linear),
                referenceManagerPtr_(referenceManagerPtr) {

        }

        bool StateConstraints::isActive(scalar_t time) const {
            return true;
        }

        vector_t StateConstraints::getValue(scalar_t time, const vector_t &state,
                                            const vector_t &input,
                                            const PreComputation &preComp) const {
            vector_t val(getNumConstraints(time));

            val << state[2] - 0.2, -state[2] + 0.5,
                    state[6] + 40.0, -state[6] + 40.0,
                    state[7] + 0.2, -state[7] + 0.2,
                    state[8] + 0.02, -state[8] + 0.02,
                    state[9] + 40.0, -state[9] + 40.0,
                    state[10] + 0.2, -state[10] + 0.2,
                    state[11] + 0.02, -state[11] + 0.02;

            return val;
        }

        VectorFunctionLinearApproximation
        StateConstraints::getLinearApproximation(scalar_t time, const vector_t &state,
                                                 const vector_t &input,
                                                 const PreComputation &preComp) const {
            VectorFunctionLinearApproximation approx;
            approx.f = getValue(time, state, input, preComp);
            approx.dfdx = matrix_t::Zero(getNumConstraints(time), state.size());
            approx.dfdx(0,2) = 1.0;
            approx.dfdx(1,2) = -1.0;
            approx.dfdx(2,6) = 1.0;
            approx.dfdx(3,6) = -1.0;
            approx.dfdx(4,7) = 1.0;
            approx.dfdx(5,7) = -1.0;
            approx.dfdx(6,8) = 1.0;
            approx.dfdx(7,8) = -1.0;
            approx.dfdx(8,9) = 1.0;
            approx.dfdx(9,9) = -1.0;
            approx.dfdx(10,10) = 1.0;
            approx.dfdx(11,10) = -1.0;
            approx.dfdx(12,11) = 1.0;
            approx.dfdx(13,11) = -1.0;

            approx.dfdu = matrix_t::Zero(getNumConstraints(time), input.size());

            return approx;
        }
    }
}