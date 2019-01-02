cmake_minimum_required(VERSION 3.5)
project(ExampleTwo)

set(CMAKE_CXX_STANDARD 11)

# 添加cmake模块
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找G2O
find_package( G2O REQUIRED )
# Eigen3
find_package(Eigen3  REQUIRED)
# OpenCV
find_package( OpenCV REQUIRED )
# 使用linear_solver_cholmod需要包含
find_package( Cholmod REQUIRED )
include_directories(
        ${G2O_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
)

SET(G2O_LIBS
        g2o_core
        g2o_stuff
        g2o_types_slam3d
        )
# Set link libraries
list(APPEND LINK_LIBS
        ${G2O_LIBS}
        ${OpenCV_LIBS}
        ${CHOLMOD_LIBRARIES}
        )

add_executable(ExampleTwo
        main.cpp
        config.cpp
        createSphere.cpp
        OptimizeSphere.cpp
        )
target_link_libraries(ExampleTwo ${LINK_LIBS})