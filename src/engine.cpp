#include "engine.hpp"
#include "sysio.hpp"
#include "utils.hpp"

// Extern variable implementation (only in engine.cpp!!! DO NOT REDECLARE
// PLEASE)
Linalg::Vec4<double> g_sunlight = Linalg::Vec4<double>(-0.1, -0.2, 0.3).unit();
Linalg::Mat<double, 4, 4> g_view_mat = Linalg::I4x4_double;
std::vector<Object3D> g_objectList;
Linalg::Vec4<double> ibuffer[MAX_VERTICES];
double sbuffer[SSHEIGHT][SSWIDTH];

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

void cast_rays();
void display();
void draw_loop() {
    double t = 0.0;
    g_view_mat = Linalg::Mat<double, 4, 4>{{1.0, 0.0, 0.0, 0.0},
                                           {0.0, 1.0, 0.0, 0.0},
                                           {0.0, 0.0, 1.0, 0.0},
                                           {0.0, 0.0, 0.0, 1.0}};
    while (1) {
        t += 0.01;
        g_objectList.at(0).transform_mat = g_objectList.at(0).transform_mat *
                                           Linalg::getRx(4.0 * 3.14 / 180.0) *
                                           Linalg::getRy(4.0 * 3.14 / 180.0);
        cast_rays();
        display();
        std::cout.flush();
    }
}

inline Linalg::Vec4<double> applyTransform(uint32_t bufferIndex,
                                           Object3D const &obj) {
    return Linalg::applyMat(
        Linalg::applyMat(ibuffer[bufferIndex], obj.transform_mat), g_view_mat);
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
                        Linalg::Vec4<double> const &p0 =
                            applyTransform(tri.at(0), obj);
                        Linalg::Vec4<double> const &p1 =
                            applyTransform(tri.at(1), obj);
                        Linalg::Vec4<double> const &p2 =
                            applyTransform(tri.at(2), obj);
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