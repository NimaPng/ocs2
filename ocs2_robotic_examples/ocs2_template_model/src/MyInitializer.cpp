//
// Created by czf on 11/4/22.
//

#include "ocs2_template_model/MyInitializer.h"

namespace ocs2 {
    namespace template_model {
        MyInitializer::MyInitializer() {
        }

        MyInitializer *MyInitializer::clone() const {
            return new MyInitializer(*this);
        }

        void MyInitializer::compute(scalar_t time, const vector_t &state, scalar_t nextTime,
                                    vector_t &input, vector_t &nextState) {
            assert(state.size() == STATE_DIM);
            nextState= state;
            input.setZero(INPUT_DIM);
            input(0)= 1.0;
            input(1) = 9.81 / 0.3;
            input(2) = 9.81 / 0.3;
        }
    }
}