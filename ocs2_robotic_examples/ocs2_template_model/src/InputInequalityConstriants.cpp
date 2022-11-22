//
// Created by czf on 11/22/22.
//

#include "ocs2_template_model/InputInequalityConstriants.h"

namespace ocs2 {
    namespace template_model {
        InputInequalityConstriants::InputInequalityConstriants(const SwitchedModelReferenceManager *referenceManagerPtr)
                :
                StateInputConstraint(ConstraintOrder::Linear),
                referenceManagerPtr_(referenceManagerPtr) {

        }

        bool InputInequalityConstriants::isActive(scalar_t time) const {
            return true;
        }

        vector_t
        InputInequalityConstriants::getValue(scalar_t time, const vector_t &state, const vector_t &input,
                                             const PreComputation &preComp) const {

            vector_t val(this->getNumConstraints(time));
            val << input[0] - min_value_alpha, -input[0] + max_value_alpha,
                    input[1] - min_value_beta, -input[1] + max_value_beta,
                    input[2] - min_value_beta, -input[2] + max_value_beta,
                    input[3] + 10.0, -input[3] + 10.0,
                    input[4] + 10.2, -input[4] + 10.2,
                    input[5] + 10.01, -input[5] + 10.05,
                    input[6] + 10.0, -input[6] + 10.0,
                    input[7] + 10.2, -input[7] + 10.2,
                    input[8] + 10.01, -input[8] + 10.05;
            return val;
        }

        VectorFunctionLinearApproximation
        InputInequalityConstriants::getLinearApproximation(scalar_t time, const vector_t &state,
                                                           const vector_t &input,
                                                           const PreComputation &preComp) const {
            VectorFunctionLinearApproximation approx;
            approx.f = getValue(time, state, input, preComp);
            approx.dfdx = matrix_t::Zero(getNumConstraints(time), state.size());
            approx.dfdu = matrix_t::Zero(getNumConstraints(time), input.size());

            approx.dfdu(0, 0) = 1.0;
            approx.dfdu(1, 0) = -1.0;
            approx.dfdu(2, 1) = 1.0;
            approx.dfdu(3, 1) = -1.0;
            approx.dfdu(4, 2) = 1.0;
            approx.dfdu(5, 2) = -1.0;

            approx.dfdu(6, 3) = 1.0;
            approx.dfdu(7, 3) = -1.0;
            approx.dfdu(8, 4) = 1.0;
            approx.dfdu(9, 4) = -1.0;
            approx.dfdu(10, 5) = 1.0;
            approx.dfdu(11, 5) = -1.0;

            approx.dfdu(12, 6) = 1.0;
            approx.dfdu(13, 6) = -1.0;
            approx.dfdu(14, 7) = 1.0;
            approx.dfdu(15, 7) = -1.0;
            approx.dfdu(16, 8) = 1.0;
            approx.dfdu(17, 8) = -1.0;

            return approx;
        }
    }
}