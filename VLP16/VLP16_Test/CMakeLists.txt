cmake_minimum_required(VERSION 2.8.7)
project(VLP16Test)
find_package(PCL 1.5 REQUIRED)

# set( CMAKE_CXX_STANDARD 14)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O3 -march=core2")

set(ROOT 		"${CMAKE_CURRENT_SOURCE_DIR}/")

include_directories(
    ${ROOT}
)

file(GLOB SOURCES
    "*.cpp"
    )

add_executable(VLP16Test ${SOURCES})
# target_link_libraries(VLP16Test ${PROJECT_LINK_LIBS})
target_link_libraries(VLP16Test ${PCL_LIBRARIES})