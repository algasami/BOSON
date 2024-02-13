#include "linalg.hpp"
#include "utils.hpp"
#include <iostream>
#include <string>
#include <vector>


constexpr uint32_t WIDTH = 80;
constexpr uint32_t HEIGHT = 50;
constexpr uint32_t SAMPLING_FACTOR = 1;

constexpr double MAX_STEP_DIST = 3.0;
constexpr double DIST_PER_STEP = 0.05;

constexpr uint32_t MAX_STEP =
    static_cast<uint32_t>(MAX_STEP_DIST / DIST_PER_STEP);
constexpr uint32_t SSWIDTH = WIDTH * SAMPLING_FACTOR;
constexpr uint32_t SSHEIGHT = HEIGHT * SAMPLING_FACTOR;

constexpr double NPZ = 1.0;

std::string LUMA_INDEX =
    "$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\\|()1{}[]?-_+~<>i!lI;:,\"^`'.";

Linalg::Vec4<double> g_sunlight = Linalg::Vec4<double>(-0.1, -0.2, 0.3).unit();

inline char getLuma(double const brightness) {
    return LUMA_INDEX.at((LUMA_INDEX.length() - 1) * brightness);
}

struct Object3D {
    std::vector<Linalg::Triangle<double>> triangles;
    Linalg::Mat<double, 4, 4> transform;
    Linalg::Vec4<double> pos;
    Linalg::Vec4<double> look;
};

std::vector<Object3D> g_objectList;

Linalg::Vec4<double> g_camera(0.0, 0.0, 0.0);

double sbuffer[SSHEIGHT][SSWIDTH];
char dbuffer[HEIGHT][WIDTH];

void initialize_objects();
void draw_loop();
void cast_rays();
void display();
int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    // testing
    Linalg::Vec4<double> v{1, 2, 3};
    std::cout << Linalg::I4x4_double * 2.11 * v << '\n';

    initialize_objects();

    // draw_loop();
    return 0;
}

void initialize_objects() // hard-coding
{

    Linalg::Vec4<double> a{0.0, 0.0, 1.3}, b{0.0, 1.0, 1.3}, c{1.0, 0.0, 1.3},
        d{0.0, 0.0, 1.6};
    g_objectList.push_back(
        Object3D{// tetrahedron
                 .triangles = {{a, b, c}, {b, c, d}, {a, c, d}, {a, b, d}}});
}

void draw_loop() {
    double t = 0.0;
    while (1) {
        t += 0.1;
        g_camera = {0.0, 0.0, 0.1 * (sin(t) + 1.0)};
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

            Linalg::Vec4<double> pos = g_camera + _raw_ray;

            bool hit = false;
            Linalg::Vec4<double> unitNormal;
            uint32_t steps = 0;
            while (steps++ < MAX_STEP && !hit) {
                for (const auto &obj : g_objectList) {
                    for (const auto &tri : obj.triangles) {
                        double dist = tri.getDist(pos);
                        if (tri.checkInside(pos)) {
                            hit = true;
                            unitNormal = tri.unit_normal;
                            break;
                        }
                    }
                    if (hit)
                        break;
                }
                pos = pos + ray_step;
            }
            if (hit) {
                sbuffer[_i][_j] = abs_generic(unitNormal.dot(g_sunlight));
            } else {
                sbuffer[_i][_j] = -0.001;
            }
        }
    }
}

inline void display() {
    for (uint32_t i = 0; i < HEIGHT; i++) {
        for (uint32_t j = 0; j < WIDTH; j++) {
            uint32_t sh = i * SAMPLING_FACTOR;
            uint32_t sw = j * SAMPLING_FACTOR;

            uint32_t count = 0;
            double brightness = 0.0;
            for (uint32_t si = sh; si < sh + SAMPLING_FACTOR; si++)
                for (uint32_t sj = sw; sj < sw + SAMPLING_FACTOR;
                     sj++, count++) {
                    brightness += sbuffer[si][sj];
                }
            if (brightness < 0.0) {
                dbuffer[i][j] = ' ';
            } else {
                dbuffer[i][j] =
                    getLuma(brightness / static_cast<double>(count));
            }
            std::cout << dbuffer[i][j];
        }
        std::cout << '\n';
    }
}