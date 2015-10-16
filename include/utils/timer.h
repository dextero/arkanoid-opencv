//
// Created by dex on 26.04.15.
//

#ifndef ARKANOID_TIMER_H
#define ARKANOID_TIMER_H

class Timer
{
public:
    Timer();

    void reset();
    double getElapsedSeconds();
    unsigned long getElapsedNanos();

private:
    struct timespec _start_time;
};

#endif //ARKANOID_TIMER_H
