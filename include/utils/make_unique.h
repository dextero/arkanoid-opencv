//
// Created by dex on 21.03.15.
//

#ifndef _PONG_MAKE_UNIQUE_H_
#define _PONG_MAKE_UNIQUE_H_

#include <bits/unique_ptr.h>

namespace std {

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}

#endif //_PONG_MAKE_UNIQUE_H_
