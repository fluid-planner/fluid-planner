cmake_minimum_required(VERSION 2.8)
project(fluid_planner CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

find_package(Eigen3 3.0.1 REQUIRED)

add_subdirectory(modules/libgp)
add_subdirectory(modules/libgp_interface)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${EIGEN3_INCLUDE_DIR}
                    ${libgp_INCLUDE_DIRS}
                    ${libgp_interface_INCLUDE_DIRS}
                    )

add_library(trajectory SHARED
            src/trajectory/trajectory.cc
            src/trajectory/state.cc
            )

add_library(${PROJECT_NAME} SHARED
            src/fluid_planner.cc
            src/cost_function/goal.cc
            src/cost_function/flow_field.cc
            )

add_executable(fluid src/main.cc)

target_Link_libraries(fluid ${PROJECT_NAME}
                      trajectory
                      gp
                      libgp_interface
                      )

# Expose this project's public includes to other subprojects through cache variable.
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
