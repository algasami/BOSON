#ifndef LINALG_HPP
#define LINALG_HPP

#include "utils.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
namespace Linalg {

template <typename T> struct Vec3 {
    T data[3];

    Vec3(T const &x, T const &y, T const &z) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }
    Vec3(Vec3 const &copy) {
        for (uint32_t i = 0; i < 3; i++)
            data[i] = copy.data[i];
    }
    Vec3() {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
    }

    friend Vec3 operator+(Vec3 const &a, Vec3 const &b) {
        return Vec3(a.data[0] + b.data[0], a.data[1] + b.data[1],
                    a.data[2] + b.data[2]);
    }
    friend Vec3 operator-(Vec3 const &a, Vec3 const &b) {
        return Vec3(a.data[0] - b.data[0], a.data[1] - b.data[1],
                    a.data[2] - b.data[2]);
    }
    friend Vec3 operator/(Vec3 const &a, T const &b) {
        return Vec3(a.data[0] / b, a.data[1] / b, a.data[2] / b);
    }
    friend Vec3 operator*(Vec3 const &a, T const &b) {
        return Vec3(a.data[0] * b, a.data[1] * b, a.data[2] * b);
    }
    friend std::ostream &operator<<(std::ostream &stream, Vec3 const &v) {
        return stream << '(' << v.data[0] << ',' << v.data[1] << ','
                      << v.data[2] << ')';
    }

    T getMag() const {
        return sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
    }

    Vec3 unit() const { return *this / getMag(); }

    T dot(Vec3 const &ref) const { return Vec3::Dot(*this, ref); }

    Vec3 cross(Vec3 const &ref) const { return Vec3::Cross(*this, ref); }

    // static functions
    static inline T Dot(Vec3 const &a, Vec3 const &b) {
        return a.data[0] * b.data[0] + a.data[1] * b.data[1] +
               a.data[2] * b.data[2];
    }
    static inline Vec3 Cross(Vec3 const &a, Vec3 const &b) {
        return Vec3(a.data[1] * b.data[2] - a.data[2] * b.data[1],
                    a.data[2] * b.data[0] - a.data[0] * b.data[2],
                    a.data[0] * b.data[1] - a.data[1] * b.data[0]);
    }
};

template <typename T> struct Triangle {
    Vec3<T> unit_normal;
    T rhs;
    Vec3<T> const p0, p1, p2;
    Triangle(Vec3<T> const &a, Vec3<T> const &b, Vec3<T> const &c)
        : p0(a), p1(b), p2(c) {
        unit_normal = (b - a).cross(c - a).unit();
        rhs = unit_normal.dot(a);
    }
    T getDist(Vec3<T> const &pos) const {
        return abs_generic(unit_normal.dot(pos) - rhs);
    }

    T checkInside(Vec3<T> const &p) const {
        Vec3<T> n1 = (p0 - p).cross(p1 - p);
        Vec3<T> n2 = (p1 - p).cross(p2 - p);
        Vec3<T> n3 = (p2 - p).cross(p0 - p);
        if (n1.getMag() == static_cast<T>(0) ||
            n2.getMag() == static_cast<T>(0) ||
            n3.getMag() == static_cast<T>(0))
            return true;
        n1 = n1.unit();
        n2 = n2.unit();
        n3 = n3.unit();
        return n1.dot(n2) > 1.0 - 0.1 && n2.dot(n3) > 1.0 - 0.1;
    }
};

/**
 * Matrix SxS
 */
template <typename T, unsigned int S> struct Mat {
    T data[S][S] = {};
    Mat() {}
    Mat(std::initializer_list<std::initializer_list<T>> d) {
        assert(d.size() == S);
        assert(d.begin()->size() == S);
        uint32_t i = 0, j = 0;
        for (std::initializer_list<T> const &dp : d) {
            j = 0;
            for (T const &v : dp) {
                data[i][j] = v;
                j++;
            }
            i++;
        }
    }

    Mat(Mat<T, S> const &copy) {
        for (uint32_t i = 0; i < S; i++) {
            for (uint32_t j = 0; j < S; j++)
                data[i][j] = copy.data[i][j];
        }
    }

    friend std::ostream &operator<<(std::ostream &os, Mat const &ref) {
        for (uint32_t i = 0; i < S; i++) {
            os << '|';
            for (uint32_t j = 0; j < S; j++)
                os << ref.data[i][j] << '\t';
            os << "|\n";
        }
        return os;
    }

    /**
     * Something like this:
     * | 1 2 3 |    | 10 |
     * | 4 5 6 | *  | 11 |
     * | 7 8 9 |    | 12 |
     */
    friend Vec3<T> operator*(Mat<T, S> const &m, Vec3<T> const &v) {
        static_assert(S >= 3, "Matrix is smaller than 3x3!");
        Vec3<T> dst = {};
        for (uint32_t r = 0; r < 3; r++)
            for (uint32_t c = 0; c < 3; c++)
                dst.data[r] += m.data[r][c] * v.data[c];
        return dst;
    }
    /**
     * | 1 2 3 |    | 1 2 3 |
     * | 4 5 6 | *  | 1 2 3 |
     * | 7 8 9 |    | 1 2 3 |
     */
    friend Mat<T, S> operator*(Mat<T, S> const &m0, Mat<T, S> const &m1) {
        Mat<T, S> dst;
        for (uint32_t r = 0; r < S; r++)
            for (uint32_t c = 0; c < S; c++)
                for (uint32_t k = 0; k < S; k++) {
                    dst.data[r][c] += m0.data[r][k] * m1.data[k][c];
                }
        return dst;
    }

    template <unsigned int S2> Mat<T, S2> toMat() {
        Mat<T, S2> dst;
        for (uint32_t i = 0; i < std::min(S, S2); i++)
            for (uint32_t j = 0; j < std::min(S, S2); j++)
                dst[i][j] = data[i][j];
        return dst;
    }
};

} // namespace Linalg

#endif