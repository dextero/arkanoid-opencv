//
// Created by dex on 17.03.15.
//

#ifndef _PONG_LEXICAL_CAST_H_
#define _PONG_LEXICAL_CAST_H_

#include <string>
#include <sstream>
#include <type_traits>

namespace {
    template<typename Dst, typename Src>
    struct LexicalCastHelper {
        static Dst cast(const Src& src) {
            Dst ret;
            std::stringstream ss;
            ss << src;
            ss >> ret;
            return ret;
        }
    };

    template<typename Src>
    struct LexicalCastHelper<std::string, Src>:
            public std::enable_if<std::is_arithmetic<Src>::value>
    {
        static std::string cast(const Src& src) {
            return std::to_string(src);
        }
    };
}

template<typename Dst, typename Src>
Dst lexical_cast(const Src& src) {
    return LexicalCastHelper<Dst, Src>::cast(src);
}

#endif //_PONG_LEXICAL_CAST_H_
