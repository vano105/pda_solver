cmake_minimum_required(VERSION 3.16)
project(pda_solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(pda_solver
    src/main.cpp
    src/parser.cpp
    src/lp.cpp
    src/simplex.cpp
    src/check.cpp
    src/bpmn.cpp
    third_party/tinyxml2.cpp
)

target_include_directories(pda_solver PRIVATE src third_party)

if(MSVC)
    target_compile_options(pda_solver PRIVATE /W4 /utf-8)
else()
    target_compile_options(pda_solver PRIVATE -Wall -Wextra)
endif()