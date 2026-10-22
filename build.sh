#!/bin/sh
# Сборка без CMake — одной командой.
g++ -std=c++17 -O2 -Wall -Wextra -Ithird_party -Isrc \
    src/*.cpp third_party/tinyxml2.cpp -o pda_solver
