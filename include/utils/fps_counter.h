//
// Created by dex on 17.03.15.
//

#ifndef _ARKANOID_FPS_COUNTER_H_
#define _ARKANOID_FPS_COUNTER_H_

#include <queue>
#include <ratio>

#include "utils/scoped_timer.h"

class FpsCounter
{
public:
    FpsCounter();

    ScopedTimer startNextFrame();
    double getFps();

private:
    static const size_t NUM_MEASURED_FRAMES = 10;

    std::queue<unsigned long> _ns_per_frame;
    unsigned long _ns_sum;
};

#endif //_ARKANOID_FPS_COUNTER_H_
