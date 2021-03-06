//
// Created by dex on 17.03.15.
//

#ifndef _ARKANOID_SCOPED_TIMER_H_
#define _ARKANOID_SCOPED_TIMER_H_

#include <functional>
#include <string>
#include "timer.h"

class ScopedTimer
{
public:
    ScopedTimer(const std::string &message = "");
    ScopedTimer(const std::function<void(unsigned long)>& callback);

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator =(const ScopedTimer&) = delete;

    ScopedTimer(ScopedTimer&&) = default;
    ScopedTimer& operator =(ScopedTimer&&) = default;

    ~ScopedTimer();

private:
    const std::string _message;
    const std::function<void(unsigned long)> _callback;
    Timer timer;
};

#endif //_ARKANOID_SCOPED_TIMER_H_
