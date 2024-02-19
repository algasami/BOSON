#pragma once

#include "linalg.hpp"
#include "sysio.hpp"
#include "utils.hpp"
#include <vector>

constexpr uint32_t WIDTH = 100;
constexpr uint32_t HEIGHT = 50;
constexpr uint32_t SAMPLING_FACTOR = 1;
constexpr uint32_t MAX_VERTICES = 1000;

constexpr double MAX_STEP_DIST = 2.0;
constexpr double DIST_PER_STEP = 0.1;

constexpr uint32_t MAX_STEP =
    static_cast<uint32_t>(MAX_STEP_DIST / DIST_PER_STEP);
constexpr uint32_t SSWIDTH = WIDTH * SAMPLING_FACTOR;
constexpr uint32_t SSHEIGHT = HEIGHT * SAMPLING_FACTOR;

constexpr double NPZ = 1.0;

Linalg::Vec4<double> g_sunlight = Linalg::Vec4<double>(-0.1, -0.2, 0.3).unit();

struct Object3D {
    std::vector<std::vector<size_t>> triangles;
    Linalg::Mat<double, 4, 4> transform_mat = Linalg::I4x4_double;
};

std::vector<Object3D> g_objectList;

Linalg::Mat<double, 4, 4> g_view_mat = Linalg::I4x4_double;

Linalg::Vec4<double> ibuffer[MAX_VERTICES];
double sbuffer[SSHEIGHT][SSWIDTH];
void initialize_objects();
void draw_loop();
void cast_rays();
void display();

void initialize_objects() // hard-coding
{
    ibuffer[0] = {0.0, 0.0, 0.0};
    ibuffer[1] = {0.0, 1.0, 0.0};
    ibuffer[2] = {0.0, 0.0, 1.0};
    ibuffer[3] = {1.0, 0.0, 0.0};
    g_objectList.push_back(Object3D{
        // tetrahedron
        .triangles = {{0, 1, 2}, {1, 2, 3}, {0, 2, 3}, {0, 1, 3}},
        .transform_mat = Linalg::Mat<double, 4, 4>{{{1.0, 0.0, 0.0, 0.0},
                                                    {0.0, 1.0, 0.0, 0.0},
                                                    {0.0, 0.0, 1.0, 2.0},
                                                    {0.0, 0.0, 0.0, 1.0}}}});
}

void draw_loop() {
    double t = 0.0;
    g_view_mat = Linalg::Mat<double, 4, 4>{{1.0, 0.0, 0.0, 0.0},
                                           {0.0, 1.0, 0.0, 0.0},
                                           {0.0, 0.0, 1.0, 0.0},
                                           {0.0, 0.0, 0.0, 1.0}};
    while (1) {
        t += 0.01;
        g_objectList.at(0).transform_mat = g_objectList.at(0).transform_mat *
                                           Linalg::getRx(8.0 * 3.14 / 180.0) *
                                           Linalg::getRy(8.0 * 3.14 / 180.0);
        cast_rays();
        display();
        std::cout.flush();
    }
}

void cast_rays() {
    for (uint32_t _i = 0; _i < SSHEIGHT; _i++) {
        for (uint32_t _j = 0; _j < SSWIDTH; _j++) {
            Linalg::Vec4<double> _raw_ray = Linalg::Vec4<double>(
                (static_cast<double>(_j) / static_cast<double>(SSWIDTH) - 0.5) *
                    2.0,
                (0.5 -
                 static_cast<double>(_i) / static_cast<double>(SSHEIGHT)) *
                    2.0,
                NPZ);
            Linalg::Vec4<double> ray_step = _raw_ray.unit() * DIST_PER_STEP;

            Linalg::Vec4<double> pos = _raw_ray;

            bool hit = false;
            Linalg::Vec4<double> unitNormal;
            uint32_t steps = 0;
            while (steps++ < MAX_STEP && !hit) {
                for (const auto &obj : g_objectList) {
                    for (const auto &tri : obj.triangles) {
                        Linalg::Vec4<double> const &p0 = Linalg::applyMat(
                            Linalg::applyMat(ibuffer[tri.at(0)],
                                             obj.transform_mat),
                            g_view_mat);
                        Linalg::Vec4<double> const &p1 = Linalg::applyMat(
                            Linalg::applyMat(ibuffer[tri.at(1)],
                                             obj.transform_mat),
                            g_view_mat);
                        Linalg::Vec4<double> const &p2 = Linalg::applyMat(
                            Linalg::applyMat(ibuffer[tri.at(2)],
                                             obj.transform_mat),
                            g_view_mat);
                        if (Linalg::checkInside(p0, p1, p2, pos)) {
                            hit = true;
                            unitNormal = Linalg::getUnitNormal(p0, p1, p2);
                            break;
                        }
                    }
                    if (hit)
                        break;
                }
                pos = pos + ray_step;
            }
            if (hit) {
                sbuffer[_i][_j] = abs_generic(
                    unitNormal.dot((g_view_mat * g_sunlight).unit()));
            } else {
                sbuffer[_i][_j] = 0.0;
            }
        }
    }
}

inline void display() {
    for (uint32_t i = 0; i < HEIGHT; i++) {
        for (uint32_t j = 0; j < WIDTH; j++) {
            double brightness = 0.0;

#if SAMPLING_FACTOR - 1U
            // TODO: UGLY FIX ^^^
            uint32_t sh = i * SAMPLING_FACTOR;
            uint32_t sw = j * SAMPLING_FACTOR;

            uint32_t count = 0;
            for (uint32_t si = sh; si < sh + SAMPLING_FACTOR; si++)
                for (uint32_t sj = sw; sj < sw + SAMPLING_FACTOR;
                     sj++, count++) {
                    brightness += sbuffer[si][sj];
                }
            brightness /= static_cast<double>(count);
#else
            brightness = sbuffer[i][j];
#endif
            if (brightness < 0.0)
                brightness = 0.0;
            std::cout << getLuma(brightness);
        }
        std::cout << '\n';
    }
}