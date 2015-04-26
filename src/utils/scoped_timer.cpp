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
}

ScopedTimer::ScopedTimer(const std::function<void(unsigned long)>& callback):
        _callback(callback)
{
}

ScopedTimer::~ScopedTimer() {
    const unsigned long diff_ns = timer.getElapsedNanos();

    if (!_message.empty()) {
        const double diff_s = (double)diff_ns / (double)std::nano::den;

        std::cerr << _message << diff_s << "s"
        << " (" << diff_ns << "ns)" << std::endl;
    }
    if (_callback) {
        _callback(diff_ns);
    }
}

