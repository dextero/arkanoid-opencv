//
// Created by dex on 17.03.15.
//

#include "utils/scoped_timer.h"

#include <cassert>
#include <iostream>
#include <ratio>

ScopedTimer::ScopedTimer(const std::string &message):
        _message(message)
{
    start();
}

ScopedTimer::ScopedTimer(const std::function<void(unsigned long)>& callback):
        _callback(callback)
{
    start();
}

ScopedTimer::~ScopedTimer() {
    const unsigned long diff_ns = getElapsedNanos();

    if (!_message.empty()) {
        const double diff_s = (double)diff_ns / (double)std::nano::den;

        std::cerr << _message << diff_s << "s"
        << " (" << diff_ns << "ns)" << std::endl;
    }
    if (_callback) {
        _callback(diff_ns);
    }
}

void ScopedTimer::start() {
    if (clock_gettime(CLOCK_REALTIME, &_start_time)) {
        throw std::runtime_error("cannot get time");
    }
}

unsigned long ScopedTimer::getElapsedNanos() {
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
