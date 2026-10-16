#pragma once
// Small fixed-size linear-algebra helpers shared by every module.

#include <array>
#include <cmath>
#include <vector>

namespace ofc {

using Vec = std::vector<double>;
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline Vec3 operator*(double s, const Vec3& a) { return {s * a[0], s * a[1], s * a[2]}; }

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

inline double norm(const Vec3& a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

inline Vec3 mul(const Mat3& M, const Vec3& a) {
    return {M[0][0] * a[0] + M[0][1] * a[1] + M[0][2] * a[2], M[1][0] * a[0] + M[1][1] * a[1] + M[1][2] * a[2],
            M[2][0] * a[0] + M[2][1] * a[1] + M[2][2] * a[2]};
}

inline Mat3 transpose(const Mat3& M) {
    return {{{M[0][0], M[1][0], M[2][0]}, {M[0][1], M[1][1], M[2][1]}, {M[0][2], M[1][2], M[2][2]}}};
}

inline double maxAbs(const Vec& v) {
    double m = 0.0;
    for (double x : v) m = std::max(m, std::fabs(x));
    return m;
}

} // namespace ofc
