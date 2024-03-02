cmake_minimum_required(VERSION 3.18)

project(bright_light)

set(CXX_STANDARD 20)


set(sources
  src/main.cpp
  src/linalg.cpp
  src/engine.cpp
)

set(headers
  src/engine.hpp
  src/linalg.hpp
  src/utils.hpp
  src/sysio.hpp
)

add_executable(executable ${sources} ${headers})

if(WIN32)
target_compile_definitions(executable PUBLIC WIN32)
elseif(LINUX)
find_package(Curses REQUIRED)
target_compile_definitions(executable PUBLIC LINUX)
target_include_directories(executable PUBLIC ${CURSES_INCLUDE_DIRS})
target_link_libraries(executable PUBLIC ${CURSES_LIBRARIES})
else()
find_package(Curses REQUIRED)
target_compile_definitions(executable PUBLIC MACOS)
target_include_directories(executable PUBLIC ${CURSES_INCLUDE_DIRS})
target_link_libraries(executable PUBLIC ${CURSES_LIBRARIES})
endif()
