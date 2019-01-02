//
// Created by tqw on 19-1-2.
//

#ifndef EXAMPLETWO_OPTIMIZESPHERE_H
#define EXAMPLETWO_OPTIMIZESPHERE_H

#include "config.h"
class OptimizeSphere{

protected:
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;
    std::string filename;// 文件名及路径
    g2o::SparseOptimizer optimizer;// 求解的优化器

public:
    OptimizeSphere(std::string filename);
    void optimize();
    void save(std::string outputFileName);
    ~OptimizeSphere()= default;
protected:
    void initOptimizer();
    void readG2oFile();
};


#endif //EXAMPLETWO_OPTIMIZESPHERE_H
