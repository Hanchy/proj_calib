cmake_minimum_required(VERSION 2.8)

# set(CMAKE_CXX_COMPILER "clang++-3.9")
set(CMAKE_CXX_FLAGS 
  "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -pedantic-errors -O0 -ggdb3")


set(DY_EXT "")
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  #set(DY_EXT "dylib")
  set(CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} /usr/local//Cellar/opencv/3.4.1_4/share/OpenCV/)
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # set(DY_EXT "so")
  set(CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} /usr/local//Cellar/opencv/3.4.1_4/share/OpenCV/)
  set(OpenCV_INCLUDE_DIR "/usr/local/include")
  set(OpenBLAS_LIBS "/opt/OpenBLAS/lib/libopenblas.so")
endif()

find_package(OpenCV REQUIRED)


set(SRC_FILES "${PROJECT_BINARY_DIR}/../src/camera.cpp"
  "${PROJECT_BINARY_DIR}/../src/read_data.cpp"
  "${PROJECT_BINARY_DIR}/../src/main.cpp"
  "${PROJECT_BINARY_DIR}/../src/bundle_adjuster.cpp"
)

find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${PROJECT_BINARY_DIR}/../include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

add_executable(3d_rex ${SRC_FILES})
target_link_libraries(3d_rex ${OpenCV_LIBS} ${OpenBLAS_LIBS}
  ${Boost_LIBRARIES} ${CERES_LIBRARIES})
