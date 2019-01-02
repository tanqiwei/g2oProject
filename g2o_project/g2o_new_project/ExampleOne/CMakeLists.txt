cmake_minimum_required(VERSION 3.5)# CMake版本号
project(ExampleOne) # 项目名

set(CMAKE_CXX_STANDARD 11)# C++标准

# 添加cmake模块
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找G2O
find_package( G2O REQUIRED )
# Eigen3
find_package(Eigen3  REQUIRED)
# OpenCV
# find_package( OpenCV REQUIRED )
include_directories(
        ${G2O_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        # ${OpenCV_DIRS}
)

# 目前只需要这两个   g2o_core g2o_stuff
SET(G2O_LIBS g2o_core g2o_stuff )
# Set link libraries
list(APPEND LINK_LIBS
        ${G2O_LIBS}
        # ${OpenCV_LIBS}
        )

add_executable(ExampleOne
        main.cpp
        CurveFittingVetex.cpp
        CurveFittingEdge.cpp
        )

target_link_libraries(ExampleOne ${LINK_LIBS})