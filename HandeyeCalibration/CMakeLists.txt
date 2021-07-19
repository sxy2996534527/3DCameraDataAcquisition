cmake_minimum_required(VERSION 2.8.7)
project(test)
find_package(PCL 1.5 REQUIRED)

set( CMAKE_CXX_STANDARD 14)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O3 -march=core2")

set(OpenCV_DIR "/home/xinyu/Downloads/opencv-3.4.14/build")
find_package(OpenCV 3 REQUIRED)

set(ROOT 		"${CMAKE_CURRENT_SOURCE_DIR}/")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${ROOT}
    ${ROOT}/include
)

file(GLOB SOURCES
    "*.cpp"
    )

link_directories(
    ${ROOT}/lib
)

set( PROJECT_LINK_LIBS
    ${OpenCV_LIBRARIES}
)

add_executable(test ${SOURCES})
target_link_libraries(test ${PROJECT_LINK_LIBS})
target_link_libraries(test ${PCL_LIBRARIES})