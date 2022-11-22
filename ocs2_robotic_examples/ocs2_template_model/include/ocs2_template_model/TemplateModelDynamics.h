//
// Created by czf on 11/3/22.
//

#ifndef SRC_TEMPLATEMODELDYNAMICS_H
#define SRC_TEMPLATEMODELDYNAMICS_H

#include "ocs2_template_model/definitions.h"
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

namespace ocs2 {
    namespace template_model {
        class TemplateModelDynamics : public SystemDynamicsBaseAD {
        public:
            /** Constructor */
            TemplateModelDynamics(const std::string &libraryFolder, bool verbose) : g(3) {
                g.setZero();
                g(2) = -9.81;
                initialize(STATE_DIM, INPUT_DIM, "template_model_dynamics", libraryFolder, true, verbose);
            }

            /** Destructor */
            ~TemplateModelDynamics() override = default;

            TemplateModelDynamics(const TemplateModelDynamics &rhs) = default;

            TemplateModelDynamics *clone() const override { return new TemplateModelDynamics(*this); }

            ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input,
                                      const ad_vector_t &parameters) const override;

        private:
            ad_vector_t g;
        };
    }
}


#endif //SRC_TEMPLATEMODELDYNAMICS_H
