cmake_minimum_required(VERSION 3.2)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
  CACHE STRING "Vcpkg toolchain file")

project(PerformanceAwareProgramming)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)
add_compile_options(
  -Wall
  -Wextra
)

find_package(fmt CONFIG REQUIRED)

add_executable(disassemble main.cpp)
target_link_libraries(disassemble PRIVATE fmt::fmt)
