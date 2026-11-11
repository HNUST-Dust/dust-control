/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <cmath>
#include <limits>
#include "chassis/balance/lqr_action_gain.h"
#include "chassis/balance/lqr_schedule.h"
int main() {
    using namespace dust_control::chassis;
    const LqrGainSchedule schedule(LqrDevelopmentSamples());
    for (double length : {.1, .15362, .2, .31101, .4}) {
        double gain[4][10]{};
        EvaluateLqrActionGain(length, gain);
        const auto reference = schedule.Evaluate(length);
        assert(reference.valid);
        for (int i=0;i<4;++i) for(int j=0;j<10;++j)
            assert(std::abs(gain[i][j]-reference.design.gain(i,j)) < 1e-9);
    }
    double invalid[4][10]{};
    EvaluateLqrActionGain(std::numeric_limits<double>::quiet_NaN(), invalid);
    for (const auto& row : invalid) for(double value : row) assert(value==0.);
}
