#ifndef LINALG_HPP
#define LINALG_HPP

#include "utils.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
namespace Linalg {

template <typename T> struct Vec4 {
    T data[4];

    Vec4(T const &x, T const &y, T const &z) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
        data[3] = 1;
    }
    Vec4(Vec4 const &copy) {
        for (uint32_t i = 0; i < 4; i++)
            data[i] = copy.data[i];
    }
    Vec4() {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 1;
    }

    friend Vec4 operator+(Vec4 const &a, Vec4 const &b) {
        return Vec4(a.data[0] + b.data[0], a.data[1] + b.data[1],
                    a.data[2] + b.data[2]);
    }
    friend Vec4 operator-(Vec4 const &a, Vec4 const &b) {
        return Vec4(a.data[0] - b.data[0], a.data[1] - b.data[1],
                    a.data[2] - b.data[2]);
    }
    friend Vec4 operator/(Vec4 const &a, T const &b) {
        return Vec4(a.data[0] / b, a.data[1] / b, a.data[2] / b);
    }
    friend Vec4 operator*(Vec4 const &a, T const &b) {
        return Vec4(a.data[0] * b, a.data[1] * b, a.data[2] * b);
    }
    friend std::ostream &operator<<(std::ostream &stream, Vec4 const &v) {
        return stream << '(' << v.data[0] << ',' << v.data[1] << ','
                      << v.data[2] << ')';
    }

    T getMag() const {
        return sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
    }

    Vec4 unit() const { return *this / getMag(); }

    T dot(Vec4 const &ref) const { return Vec4::Dot(*this, ref); }

    Vec4 cross(Vec4 const &ref) const { return Vec4::Cross(*this, ref); }

    // static functions
    static inline T Dot(Vec4 const &a, Vec4 const &b) {
        return a.data[0] * b.data[0] + a.data[1] * b.data[1] +
               a.data[2] * b.data[2];
    }
    static inline Vec4 Cross(Vec4 const &a, Vec4 const &b) {
        return Vec4(a.data[1] * b.data[2] - a.data[2] * b.data[1],
                    a.data[2] * b.data[0] - a.data[0] * b.data[2],
                    a.data[0] * b.data[1] - a.data[1] * b.data[0]);
    }
};

template <typename T> struct Triangle {
    Vec4<T> unit_normal;
    T rhs;
    Vec4<T> const p0, p1, p2;
    Triangle(Vec4<T> const &a, Vec4<T> const &b, Vec4<T> const &c)
        : p0(a), p1(b), p2(c) {
        unit_normal = (b - a).cross(c - a).unit();
        rhs = unit_normal.dot(a);
    }
    T getDist(Vec4<T> const &pos) const {
        return abs_generic(unit_normal.dot(pos) - rhs);
    }

    T checkInside(Vec4<T> const &p) const {
        Vec4<T> n1 = (p0 - p).cross(p1 - p);
        Vec4<T> n2 = (p1 - p).cross(p2 - p);
        Vec4<T> n3 = (p2 - p).cross(p0 - p);
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
template <typename T, unsigned int S, unsigned int S1> struct Mat {
    T data[S][S1] = {};
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

    Mat(Mat<T, S, S1> const &copy) {
        for (uint32_t i = 0; i < S; i++) {
            for (uint32_t j = 0; j < S1; j++)
                data[i][j] = copy.data[i][j];
        }
    }

    friend std::ostream &operator<<(std::ostream &os, Mat const &ref) {
        for (uint32_t i = 0; i < S; i++) {
            os << '|';
            for (uint32_t j = 0; j < S1; j++)
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
    friend Vec4<T> operator*(Mat<T, S, S1> const &m, Vec4<T> const &v) {
        static_assert(S >= 4 && S1 >= 4, "Matrix is smaller than 4x4!");
        Vec4<T> dst = {};
        for (uint32_t r = 0; r < 4; r++)
            for (uint32_t c = 0; c < 4; c++)
                dst.data[r] += m.data[r][c] * v.data[c];
        return dst;
    }
    /**
     * | 1 2 3 |    | 1 2 3 |
     * | 4 5 6 | *  | 1 2 3 |
     * | 7 8 9 |    | 1 2 3 |
     */
    // !! Support for Mat S0*S1 mult S1*S2 = Mat S0*S2
    template <unsigned int S2>
    friend Mat<T, S, S2> operator*(Mat<T, S, S1> const &m0,
                                   Mat<T, S1, S2> const &m1) {
        Mat<T, S, S2> dst;
        for (uint32_t r = 0; r < S; r++)
            for (uint32_t c = 0; c < S2; c++)
                for (uint32_t k = 0; k < S1; k++) {
                    dst.data[r][c] += m0.data[r][k] * m1.data[k][c];
                }
        return dst;
    }

    friend Mat<T, S, S1> operator*(Mat<T, S, S1> const &m0, T const &x) {
        Mat<T, S, S1> dst;
        for (uint32_t r = 0; r < S; r++)
            for (uint32_t c = 0; c < S1; c++)
                dst.data[r][c] = m0.data[r][c] * x;
        return dst;
    }
};

Mat<double, 4, 4> I4x4_double{
    {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

} // namespace Linalg

#endif