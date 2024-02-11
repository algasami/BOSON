cmake_minimum_required(VERSION 3.18)

project(bright_light)

set(CXX_STANDARD 20)


set(sources
  src/main.cpp
)

set(headers
  src/linalg.hpp
  src/utils.hpp
)

add_executable(executable ${sources} ${headers})