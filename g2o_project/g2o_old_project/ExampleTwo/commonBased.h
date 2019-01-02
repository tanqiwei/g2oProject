//
// Created by tqw on 18-12-26.
//

#ifndef EXAMPLEONE_COMMONBASED_H
#define EXAMPLEONE_COMMONBASED_H

// C++流控制
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>//计算流逝时间
#include <vector>
#include <cmath>

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// 引入g2o
// 引入矩阵块
#include <g2o/core/block_solver.h>
// 线性求解器
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
// 三种优化方式
#include <g2o/core/optimization_algorithm_dogleg.h>//Dogleg
#include <g2o/core/optimization_algorithm_gauss_newton.h>//GN
#include <g2o/core/optimization_algorithm_levenberg.h> // LM

// 节点类
#include <g2o/types/slam3d/vertex_se3.h>
// 边
#include <g2o/types/slam3d/edge_se3.h>
//　随机采样器
#include <g2o/stuff/sampler.h>
//　工厂
#include <g2o/core/factory.h>
// Eigen3
#include <Eigen/Core>
#include <Eigen/StdVector>




#endif //EXAMPLEONE_COMMONBASED_H
