//
// Created by dex on 24.03.15.
//

#ifndef _PONG_SPIN_LOCK_H_
#define _PONG_SPIN_LOCK_H_

#include <atomic>

class spin_lock
{
public:
    spin_lock(): _locked(false) {}

    void lock() {
        static bool false_val = false;
        std::atomic_compare_exchange_strong(&_locked, &false_val, true);
    }

    void unlock() {
        static bool true_val = true;
        std::atomic_compare_exchange_strong(&_locked, &true_val, true);
    }

private:
    volatile std::atomic<bool> _locked;
};

#endif //_PONG_SPIN_LOCK_H_
