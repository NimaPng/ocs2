//
// Created by czf on 11/3/22.
//

#include "ocs2_template_model/TemplateModelDynamics.h"
#include "ocs2_core/Types.h"

ocs2::ad_vector_t
ocs2::template_model::TemplateModelDynamics::systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t &state,
                                                           const ocs2::ad_vector_t &input,
                                                           const ocs2::ad_vector_t &parameters) const {

    const ad_scalar_t &alpha = input(0);
    const ad_scalar_t &beta1 = input(1);
    const ad_scalar_t &beta2 = input(2);
//    const ad_scalar_t alpha(1);
//    const ad_scalar_t beta1(30.0);
//    const ad_scalar_t beta2(30.0);

    const ad_vector_t &u1 = input.segment(3, 3);
    const ad_vector_t &u2 = input.segment(6, 3);

    const ad_vector_t &c = state.head(3);
    const ad_vector_t &cdot = state.segment(3, 3);
    const ad_vector_t &z1 = state.segment(6, 3);
    const ad_vector_t &z2 = state.segment(9, 3);

    // dxdt
    ad_vector_t stateDerivative(STATE_DIM);
    stateDerivative << alpha * cdot, beta1 * (c - z1) + beta2 * (c - z2) + alpha * g, u1, u2;

    return stateDerivative;
}
