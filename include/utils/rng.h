//
// Created by dex on 26.04.15.
//

#ifndef ARKANOID_RNG_H
#define ARKANOID_RNG_H

#include <random>

template<typename ValueT,
         typename Distribution = std::uniform_int_distribution<ValueT>,
         typename Generator = std::mt19937,
         typename RandomDevice = std::random_device>
class RNG
{
public:
    RNG(ValueT min, ValueT max):
        _device(),
        _generator(_device()),
        _distribution(min, max)
    { }

    inline ValueT next()
    {
        return _distribution(_generator);
    }

private:
    RandomDevice _device;
    Generator _generator;
    Distribution _distribution;
};

#endif //ARKANOID_RNG_H
