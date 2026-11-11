/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <algorithm>
#include <cmath>
#include "chassis/balance/generated/lqr14_gain_samples.h"
namespace dust_control::chassis {
// Actions use only the original state subset; all added state/input channels are masked.
inline void EvaluateLqr14ActionGain(double length, double gain[4][10]) {
    for (int i=0;i<4;++i) for(int j=0;j<10;++j) gain[i][j]=0.;
    if (!std::isfinite(length)) return;
    const auto& samples=Lqr14DevelopmentSamples();
    length=std::clamp(length,samples.front().leg_length_m,samples.back().leg_length_m);
    for(size_t k=1;k<samples.size();++k) if(length<=samples[k].leg_length_m) {
        const auto& a=samples[k-1];const auto& b=samples[k];
        const double t=(length-a.leg_length_m)/(b.leg_length_m-a.leg_length_m);
        for(int i=0;i<4;++i) for(int j=0;j<10;++j) gain[i][j]=(1-t)*a.gain(i,j)+t*b.gain(i,j);
        return;
    }
}
}
