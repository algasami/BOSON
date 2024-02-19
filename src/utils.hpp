#pragma once
#include <cstdint>
#include <cstring>
template <typename T> inline T abs_generic(const T &a) {
    if (a >= 0)
        return a;
    return -a;
}

constexpr char LUMA_INDEX[] =
    "$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\\|()1{}[]?-_+~<>i!lI;:,\"^`'.";

inline char getLuma(double const brightness) {
    return LUMA_INDEX[static_cast<uint32_t>(
        static_cast<double>(strlen(LUMA_INDEX) - 1) * brightness)];
}