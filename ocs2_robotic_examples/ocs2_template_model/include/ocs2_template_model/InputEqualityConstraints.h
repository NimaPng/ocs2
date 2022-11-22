//
// Created by czf on 11/22/22.
//

#ifndef SRC_INPUTEQUALITYCONSTRAINTS_H
#define SRC_INPUTEQUALITYCONSTRAINTS_H

#include <ocs2_core/constraint/StateInputConstraint.h>
#include "ocs2_template_model/SwitchedModelReferenceManager.h"

namespace ocs2 {
    namespace template_model {
        class InputEqualityConstraints : public StateInputConstraint {
        public:
            InputEqualityConstraints(const SwitchedModelReferenceManager *referenceManagerPtr);

            ~InputEqualityConstraints() override = default;

            InputEqualityConstraints *
            clone() const override { return new InputEqualityConstraints(*this); }

            bool isActive(scalar_t time) const override;

            size_t getNumConstraints(scalar_t time) const override { return 4; }

            vector_t getValue(scalar_t time, const vector_t &state, const vector_t &input,
                              const PreComputation &preComp) const override;

            VectorFunctionLinearApproximation
            getLinearApproximation(scalar_t time, const vector_t &state, const vector_t &input,
                                   const PreComputation &preComp) const override;

        private:
            const SwitchedModelReferenceManager *referenceManagerPtr_;
        };
    }
}

#endif //SRC_INPUTEQUALITYCONSTRAINTS_H
