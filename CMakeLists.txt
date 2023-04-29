cmake_minimum_required(VERSION 3.26)
project(PerformanceAwareProgramming)

include(FetchContent)
include(ExternalProject)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
  -Wall
  -Wextra
)

FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG master
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(fmt)

add_executable(disassemble main.cpp)
target_link_libraries(disassemble PRIVATE fmt)
