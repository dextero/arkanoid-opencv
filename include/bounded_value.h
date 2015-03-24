//
// Created by dex on 24.03.15.
//

#ifndef _PONG_BOUNDED_VALUE_H_
#define _PONG_BOUNDED_VALUE_H_

template<typename T, T Min, T Max>
struct BoundedValue
{
    T value;

    BoundedValue(T val): value(val) {}

    BoundedValue& operator +=(T add) {
        auto new_val = value + add;
        if (new_val > Max) {
            new_val = Max;
        }

        value = new_val;
        return *this;
    }
    BoundedValue& operator -=(T sub) {
        auto new_val = value - sub;
        if (new_val < Min) {
            new_val = Min;
        }

        value = new_val;
        return *this;
    }
    BoundedValue& operator =(T new_val) {
        value = new_val;
        return *this;
    }
    operator T() const { return value; }
};

#endif //_PONG_BOUNDED_VALUE_H_
