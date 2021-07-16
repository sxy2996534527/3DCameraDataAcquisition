# CMakeLists files in this project can
# refer to the root source directory of the project as ${MinimalPclExample_SOURCE_DIR} and
# to the root binary directory of the project as ${MinimalPclExample_BINARY_DIR}.
cmake_minimum_required (VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project (MinimalPclExample)

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")


if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Files
    ${MinimalPclExample_SOURCE_DIR}/MinimalPclExample.cpp
    ${MinimalPclExample_SOURCE_DIR}/ReadMe.txt
)

add_executable (MinimalPclExample
    ${Files}
)

# set(ThirdPartyDIR "/path/to/your/3rd/party/libraries/directory")

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
# find_package(PCL 1.8.0 EXACT PATHS "${ThirdPartyDIR}/pcl-1.8.0/cmake" "${ThirdPartyDIR}/pcl-1.8.0/share/pcl-1.8")
find_package(PCL 1.7 REQUIRED)
# find_package(VTK REQUIRED)

if (UNIX AND NOT APPLE)
    target_link_libraries(MinimalPclExample
        ${PHOXI_LIBRARY}
        rt
    )
else (UNIX AND NOT APPLE)
    target_link_libraries(MinimalPclExample
        ${PHOXI_LIBRARY}
    )
endif(UNIX AND NOT APPLE)

get_target_property(ReleaseBinFolder MinimalPclExample RUNTIME_OUTPUT_DIRECTORY_RELEASE)
get_target_property(DebugBinFolder MinimalPclExample RUNTIME_OUTPUT_DIRECTORY_DEBUG)

if(MSVC)
    add_custom_command(TARGET MinimalPclExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_DEBUG}"
            $<TARGET_FILE_DIR:MinimalPclExample>
    )

    add_custom_command(TARGET MinimalPclExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_RELEASE}"
            $<TARGET_FILE_DIR:MinimalPclExample>
    )
endif(MSVC)

set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")
set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.11")
# set(VTK_DIR "PATH/TO/VTK/BUILD/DIRECTORY")
# SET(VTK_DIR "/usr/include/vtk-7.1" CACHE PATH "VTK directory override" FORCE)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_include_directories (MinimalPclExample PUBLIC ${PHOXI_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
target_link_libraries (MinimalPclExample ${PCL_LIBRARIES})

set_target_properties(MinimalPclExample
    PROPERTIES
    #for executables, inheritance of variables CMAKE_DEBUG_POSTFIX... does not work
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)
