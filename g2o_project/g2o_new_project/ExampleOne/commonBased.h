//
// Created by tqw on 18-12-26.
//

#ifndef EXAMPLEONE_COMMONBASED_H
#define EXAMPLEONE_COMMONBASED_H

// C++流控制
#include <iostream>
#include <chrono>//计算流逝时间

// 为生成高斯随机数而引入的opencv
//#include <opencv2/core.hpp>

// 引入g2o
// 引入矩阵块
#include <g2o/core/block_solver.h>
// 线性求解器
#include <g2o/solvers/dense/linear_solver_dense.h>
// 三种优化方式
#include <g2o/core/optimization_algorithm_dogleg.h>//Dogleg
#include <g2o/core/optimization_algorithm_gauss_newton.h>//GN
#include <g2o/core/optimization_algorithm_levenberg.h> // LM

// 节点类
#include <g2o/core/base_vertex.h>
// 边
#include <g2o/core/base_unary_edge.h> // 模拟一元超边(unary hyper-edge)的模板类
//　随机采样器
#include <g2o/stuff/sampler.h>

#endif //EXAMPLEONE_COMMONBASED_H
