cmake_minimum_required(VERSION 3.4 FATAL_ERROR)
project(PyBinding)


#add_subdirectory(venv)
#include_directories(${PYTHON_INCLUDE_DIRS})
find_package(PCL 1.9 REQUIRED COMPONENTS common io)
find_package(pybind11 REQUIRED)
#add_subdirectory(pybind11)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



# link other sub modules
add_library(common STATIC pointCloud.cpp)

set_target_properties(
        common
        PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        PREFIX ""
        OUTPUT_NAME "better_name"


)

pybind11_add_module(pcl pybind_pcl.cpp)

target_link_libraries(pcl PRIVATE common)

set_target_properties(
        pcl
        PROPERTIES
        PREFIX ""
        OUTPUT_NAME "pcl"
        LIBRARY_OUTPUT_DIRECTORY /home/haochen/Projects/pybinding/lib

)


