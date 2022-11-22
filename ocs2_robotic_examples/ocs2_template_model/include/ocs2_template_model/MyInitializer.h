//
// Created by czf on 11/4/22.
//

#ifndef SRC_MYINITIALIZER_H
#define SRC_MYINITIALIZER_H


#include <ocs2_core/initialization/Initializer.h>
#include "ocs2_template_model/definitions.h"

namespace ocs2 {
    namespace template_model {
        class MyInitializer final : public Initializer {
        public:
            MyInitializer();

            ~MyInitializer() override = default;

            MyInitializer *clone() const override;

            void compute(scalar_t time, const vector_t &state, scalar_t nextTime, vector_t &input,
                         vector_t &nextState) override;

        private:
            MyInitializer(const MyInitializer &other) = default;
        };
    }
}
#endif //SRC_MYINITIALIZER_H
