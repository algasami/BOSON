#ifndef LINALG_HPP
#define LINALG_HPP

#include "utils.hpp"
#include <cmath>
#include <iostream>
namespace Linalg {

template <typename T> struct Vec3 {
    T x, y, z;

    Vec3(T x, T y, T z) : x(x), y(y), z(z) {}
    Vec3(Vec3 const &copy) {
        x = copy.x;
        y = copy.y;
        z = copy.z;
    }
    Vec3() {
        x = static_cast<T>(0);
        y = static_cast<T>(0);
        z = static_cast<T>(0);
    }

    friend Vec3 operator+(Vec3 const &a, Vec3 const &b) {
        return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    friend Vec3 operator-(Vec3 const &a, Vec3 const &b) {
        return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
    }
    friend Vec3 operator/(Vec3 const &a, T const &b) {
        return Vec3(a.x / b, a.y / b, a.z / b);
    }
    friend Vec3 operator*(Vec3 const &a, T const &b) {
        return Vec3(a.x * b, a.y * b, a.z * b);
    }
    friend std::ostream &operator<<(std::ostream &stream, Vec3 const &v) {
        return stream << '(' << v.x << ',' << v.y << ',' << v.z << ')';
    }

    T getMag() const { return sqrt(x * x + y * y + z * z); }

    Vec3 unit() const { return *this / getMag(); }

    T dot(Vec3 const &ref) const { return Vec3::Dot(*this, ref); }

    Vec3 cross(Vec3 const &ref) const { return Vec3::Cross(*this, ref); }

    // static functions
    static inline T Dot(Vec3 const &a, Vec3 const &b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    static inline Vec3 Cross(Vec3 const &a, Vec3 const &b) {
        return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x);
    }
};

template <typename T> class Triangle {
    // un.x * x + un.y * y + un.z * z = rhs
  public:
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

} // namespace Linalg

#endif