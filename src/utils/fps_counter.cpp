//
// Created by dex on 17.03.15.
//

#include "utils/fps_counter.h"

FpsCounter::FpsCounter():
        _ns_sum(0UL)
{}

ScopedTimer FpsCounter::startNextFrame() {
    return ScopedTimer([this](unsigned long dt_ns) {
        _ns_per_frame.push(dt_ns);
        if (_ns_per_frame.size() > NUM_MEASURED_FRAMES) {
            _ns_sum -= _ns_per_frame.front();
            _ns_per_frame.pop();
        }

        _ns_sum += dt_ns;
    });
}

double FpsCounter::getFps() {
    if (_ns_per_frame.size() == 0) {
        return 0.0;
    }

    return (double)std::nano::den / (double)(_ns_sum / _ns_per_frame.size());
}
