#include "engine.hpp"
#include <iostream>

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    initialize_objects();

    draw_loop();
    return 0;
}
