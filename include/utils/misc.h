//
// Created by dex on 26.04.15.
//

#ifndef ARKANOID_MISC_H
#define ARKANOID_MISC_H

template<typename T>
T clamp(T val, T min_val, T max_val)
{
    return std::max(min_val,
                    std::min(max_val, val));
}

#endif //ARKANOID_MISC_H
