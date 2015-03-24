//
// Created by dex on 17.03.15.
//

#ifndef _PONG_SCOPED_TIMER_H_
#define _PONG_SCOPED_TIMER_H_

#include <functional>
#include <string>

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
    struct timespec _start_time;

    void start();
    unsigned long getElapsedNanos();
};

#endif //_PONG_SCOPED_TIMER_H_
