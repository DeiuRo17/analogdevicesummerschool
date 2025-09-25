cmake_minimum_required(VERSION 3.31.6 )
project(buffer_plot)

if(POLICY CMP0169)
cmake_policy(SET CMP0169 OLD)
endif()

include(FetchContent)
FetchContent_Declare(
        matplotlib_cpp
        GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
        GIT TAG master
)

FetchContent_Populate(matplotlib_cpp)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Embed)
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp SYSTEM INTERFACE ${matplotlib_cpp_SOURCE_DIR})

target_link_libraries(matplotlib_cpp INTERFACE Python3::Python)

add_executable(main main.cpp)
target_link_libraries(main iio matplotlib_cpp)         
target_compile_definitions(main PRIVATE WITHOUT_NUMPY)