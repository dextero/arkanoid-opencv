//
// Created by dex on 26.04.15.
//

#include <stdexcept>
#include <bits/time.h>
#include <assert.h>
#include "utils/timer.h"
#include <ratio>

Timer::Timer()
{
    reset();
}

void Timer::reset() {
    if (clock_gettime(CLOCK_REALTIME, &_start_time)) {
        throw std::runtime_error("cannot get time");
    }
}

double Timer::getElapsedSeconds()
{
    unsigned long nanos = getElapsedNanos();
    return (double)nanos / std::nano::den;
}

unsigned long Timer::getElapsedNanos() {
    struct timespec end_time;

    if (clock_gettime(CLOCK_REALTIME, &end_time)) {
        throw std::runtime_error("cannot get time");
    }

    const long diff_s = end_time.tv_sec - _start_time.tv_sec;
    const long diff_ns = end_time.tv_nsec - _start_time.tv_nsec;
    long time_diff_ns = diff_s * std::nano::den + diff_ns;

    assert(time_diff_ns >= 0);
    return (unsigned long)time_diff_ns;
}
