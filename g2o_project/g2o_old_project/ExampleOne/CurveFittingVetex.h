//
// Created by tqw on 18-12-26.
// 曲线模型的顶点,模板参数,优化变量维度和数据类型
//

#ifndef EXAMPLEONE_CURVEFITTINGVETEX_H
#define EXAMPLEONE_CURVEFITTINGVETEX_H


#include "commonBased.h"

class CurveFittingVetex: public g2o::BaseVertex<3,Eigen::Vector3d>
{
    // BaseVertex的两个模板,第一个是模板的最小维度(比如三维旋转空间是3）,第二个是表达估计的内部类型(比如四元数)
public:
    // Eigen的使用时内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl();
    virtual void oplusImpl(const double* update);
    // 读写函数
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const ;
};


#endif //EXAMPLEONE_CURVEFITTINGVETEX_H
