#pragma once

#include "linalg.hpp"
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

struct Object3D {
    std::vector<std::vector<size_t>> triangles;
    Linalg::Mat<double, 4, 4> transform_mat = Linalg::I4x4_double;
};

extern Linalg::Vec4<double> g_sunlight;

extern std::vector<Object3D> g_objectList;

extern Linalg::Mat<double, 4, 4> g_view_mat;

extern Linalg::Vec4<double> ibuffer[MAX_VERTICES];
extern double sbuffer[SSHEIGHT][SSWIDTH];

void initialize_objects();
void draw_loop();
