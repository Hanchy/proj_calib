cmake_minimum_required(VERSION 2.8)

# set(CMAKE_CXX_COMPILER "clang++-3.9")
set(CMAKE_CXX_FLAGS 
  "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic-errors -O0 -ggdb3")

set(OpenCV_INCLUDE_DIR "/usr/local/include")
set(OpenCV_LIBS "/usr/local/lib/libopencv_core.so"
  "/usr/local/lib/libopencv_calib3d.so"
  "/usr/local/lib/libopencv_highgui.so"
  "/usr/local/lib/libopencv_imgproc.so"
  "/usr/local/lib/libopencv_viz.so")

set(OpenBLAS_LIBS "/opt/OpenBLAS/lib/libopenblas.so")

set(SRC_FILES "${PROJECT_BINARY_DIR}/../src/camera.cpp"
  #"${PROJECT_BINARY_DIR}/../src/SpacePoints.cpp"
  "${PROJECT_BINARY_DIR}/../src/read_data.cpp"
  "${PROJECT_BINARY_DIR}/../src/main.cpp"
  "${PROJECT_BINARY_DIR}/../src/bundle_adjuster.cpp"
)

find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${PROJECT_BINARY_DIR}/../include)
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

add_executable(3d_rex ${SRC_FILES})
target_link_libraries(3d_rex ${OpenCV_LIBS} ${OpenBLAS_LIBS}
  ${Boost_LIBRARIES} ${CERES_LIBRARIES})
