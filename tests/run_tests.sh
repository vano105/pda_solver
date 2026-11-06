#!/bin/sh
# Сборка и запуск тестов. Запускать из корня проекта: sh tests/run_tests.sh
set -e
g++ -std=c++17 -O1 -Wall -Wextra -Ithird_party -Isrc \
    tests/test_main.cpp \
    src/parser.cpp src/lp.cpp src/dw.cpp src/simplex.cpp src/check.cpp src/bpmn.cpp \
    third_party/tinyxml2.cpp -o pda_tests
./pda_tests