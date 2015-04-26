//
// Created by dex on 26.04.15.
//

#ifndef PONG_MISC_H
#define PONG_MISC_H

template<typename T>
T clamp(T val, T min_val, T max_val)
{
    return std::max(min_val,
                    std::min(max_val, val));
}

#endif //PONG_MISC_H
