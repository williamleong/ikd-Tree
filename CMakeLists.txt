cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ikd_tree_demo)

add_compile_options(-std=c++14)
# set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3")
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread")
set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, e.g. Debug, Release, RelWithDebInfo, MinSizeRel.")

find_package(PCL 1.8 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ikd-Tree
)

# add_executable(ikd_tree_demo examples/ikd_Tree_demo.cpp ikd-Tree/ikd_Tree.cpp)
# target_link_libraries(ikd_tree_demo ${PCL_LIBRARIES})

# add_executable(ikd_Tree_Async_demo examples/ikd_Tree_Async_demo.cpp ikd-Tree/ikd_Tree.cpp)
# target_link_libraries(ikd_Tree_Async_demo ${PCL_LIBRARIES})

add_executable(ikd_Tree_Search_demo examples/ikd_Tree_Search_demo.cpp ikd-Tree/ikd_Tree.cpp)
target_link_libraries(ikd_Tree_Search_demo ${PCL_LIBRARIES})