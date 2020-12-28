cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
)

## Add executable
add_executable(calibrate
  src/camera_calibration.cpp
)

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
  src/student_interface.cpp
  src/find_objects.hpp
  src/find_objects.cpp
  src/planning/planning.hpp
  src/planning/planning.cpp
  src/planning/voronoi.hpp
  src/planning/voronoi.cpp
  src/planning/dubins.hpp
  src/planning/dubins.cpp
  src/planning/dubins_multipoint.cpp
  src/planning/dubins_multipoint.hpp
  libs/boost/polygon/voronoi.hpp
  libs/boost/geometry.hpp
  libs/boost/graph/graph_traits.hpp
  libs/boost/graph/adjacency_list.hpp
  libs/boost/graph/dijkstra_shortest_paths.hpp
  libs/clipper/clipper.hpp
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}  
)
