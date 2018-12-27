//
// Created by tqw on 18-12-26.
//

#ifndef EXAMPLEONE_CURVEFITTINGEDGE_H
#define EXAMPLEONE_CURVEFITTINGEDGE_H

#include "CurveFittingVetex.h"

class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVetex>
{
    //   一元超边,即只连接一个顶点,要给出三个,一个是维度,一个是变量类型,一个是连接的顶点类型
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 构造函数
    CurveFittingEdge (double x):g2o::BaseUnaryEdge<1,double,CurveFittingVetex>(),_x (x) {};
    // 计算曲线模型误差
    void computeError();
    //存取
    bool read(std::istream& is);
    bool write(std::ostream& os) const;

public:
    double _x;//x值,y值为_measurement
};




#endif //EXAMPLEONE_CURVEFITTINGEDGE_H
