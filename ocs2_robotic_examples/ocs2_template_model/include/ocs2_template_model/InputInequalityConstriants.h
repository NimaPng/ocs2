//
// Created by czf on 11/22/22.
//

#ifndef SRC_INPUTINEQUALITYCONSTRIANTS_H
#define SRC_INPUTINEQUALITYCONSTRIANTS_H

#include <ocs2_core/constraint/StateInputConstraint.h>
#include "ocs2_template_model/SwitchedModelReferenceManager.h"

namespace ocs2 {
    namespace template_model {
        class InputInequalityConstriants : public StateInputConstraint {
        public:
            InputInequalityConstriants(const SwitchedModelReferenceManager *referenceManagerPtr);

            ~InputInequalityConstriants() override = default;

            InputInequalityConstriants *
            clone() const override { return new InputInequalityConstriants(*this); }

            bool isActive(scalar_t time) const override;

            size_t getNumConstraints(scalar_t time) const override { return 18; }

            vector_t getValue(scalar_t time, const vector_t &state, const vector_t &input,
                              const PreComputation &preComp) const override;

            VectorFunctionLinearApproximation
            getLinearApproximation(scalar_t time, const vector_t &state, const vector_t &input,
                                   const PreComputation &preComp) const override;

        private:
            const SwitchedModelReferenceManager *referenceManagerPtr_;

            scalar_t min_value_alpha = 0.2;
            scalar_t max_value_alpha = 5.0;
            scalar_t min_value_beta = 25.0;
            scalar_t max_value_beta = 35.0;
        };
    }
}

#endif //SRC_INPUTINEQUALITYCONSTRIANTS_H
