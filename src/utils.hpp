#ifndef UTILS_HPP
#define UTILS_HPP
template <typename T> inline T abs_generic(const T &a) {
    if (a >= 0)
        return a;
    return -a;
}
#endif