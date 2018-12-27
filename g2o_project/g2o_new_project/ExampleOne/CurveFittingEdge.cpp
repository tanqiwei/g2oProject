//
// Created by tqw on 18-12-27.
//

#include "CurveFittingEdge.h"

void CurveFittingEdge::computeError()
{
    // 一元超边,超边存储的节点为VertexContainer类型,你可以把它当成一个数组,取第一个就是0,对于一元边来说,这就是它的
    // 连接的边
    const CurveFittingVetex *v = static_cast<const CurveFittingVetex*>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    // _error是传入时定义的误差ErrorVector,维度D是构建CurveFittingEdge已经定义,必然是个向量,只不过是Eigen::Matrix的_Cols为1
    _error(0,0) = _measurement-std::exp(abc[0]*pow(_x,2)+abc[1]*_x+abc[2]);// y-f(x)=error
}
bool CurveFittingEdge::read(std::istream &is)
{
    is >> _x;
    return true;
}
bool CurveFittingEdge::write(std::ostream &os) const
{
    os << _x;
    return os.good();
}