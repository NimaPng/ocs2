//
// Created by czf on 11/22/22.
//

#include "ocs2_template_model/InputEqualityConstraints.h"

namespace ocs2 {
    namespace template_model {
        InputEqualityConstraints::InputEqualityConstraints(const SwitchedModelReferenceManager *referenceManagerPtr) :
                StateInputConstraint(ConstraintOrder::Linear),
                referenceManagerPtr_(referenceManagerPtr) {

        }

        bool InputEqualityConstraints::isActive(scalar_t time) const {
            return true;
        }

        vector_t
        InputEqualityConstraints::getValue(scalar_t time, const vector_t &state, const vector_t &input,
                                           const PreComputation &preComp) const {
            auto flags = referenceManagerPtr_->getFlag(time);
            vector_t val(this->getNumConstraints(time));
            if (flags[0] && !flags[1]) {
                val << input[2], input.segment(3, 3);
            } else if (!flags[0] && flags[1]) {
                val << input[1], input.segment(6, 3);
            } else {
                val.setZero();
            }
            return val;
        }

        VectorFunctionLinearApproximation
        InputEqualityConstraints::getLinearApproximation(scalar_t time, const vector_t &state,
                                                         const vector_t &input,
                                                         const PreComputation &preComp) const {
            VectorFunctionLinearApproximation approx;
            approx.f = getValue(time, state, input, preComp);
            approx.dfdx = matrix_t::Zero(getNumConstraints(time), state.size());
            approx.dfdu = matrix_t::Zero(getNumConstraints(time), input.size());

            auto flags = referenceManagerPtr_->getFlag(time);
            if (flags[0] && !flags[1]) {
                approx.dfdu(0, 2) = 1.0;
                approx.dfdu.block<3, 3>(1, 3).setIdentity();
            } else if (!flags[0] && flags[1]) {
                approx.dfdu(0, 1) = 1.0;
                approx.dfdu.block<3, 3>(1, 6).setIdentity();
            }
            return approx;
        }
    }
}
