//
// Created by czf on 11/2/22.
//

#ifndef SRC_STATECONSTRAINTS_H
#define SRC_STATECONSTRAINTS_H

#include <ocs2_core/constraint/StateInputConstraint.h>
#include "ocs2_template_model/SwitchedModelReferenceManager.h"

namespace ocs2 {
    namespace template_model {
        class StateConstraints : public StateInputConstraint {
        public:
            StateConstraints(const SwitchedModelReferenceManager *referenceManagerPtr);

            ~StateConstraints() override = default;

            StateConstraints *
            clone() const override { return new StateConstraints(*this); }

            bool isActive(scalar_t time) const override;

            size_t getNumConstraints(scalar_t time) const override { return 14; }

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


#endif //SRC_STATECONSTRAINTS_H
