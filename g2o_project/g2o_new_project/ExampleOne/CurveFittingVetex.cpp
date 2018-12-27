//
// Created by tqw on 18-12-26.
//
#include "commonBased.h"
#include "CurveFittingVetex.h"

void CurveFittingVetex::setToOriginImpl()  // 重置
{// 顶点内部状态初始值
    _estimate << 0,0,0;
}

void CurveFittingVetex::oplusImpl(const double *update) // 该方法应该应用一个增量用于更新估计参数$\oplus$
{
    _estimate += Eigen::Vector3d(update);
}

bool CurveFittingVetex::read(std::istream &is)
{
    is >> _estimate[0] >> _estimate[1] >> _estimate[2];
    return true;
}
bool CurveFittingVetex::write(std::ostream &os) const
{
    os <<  _estimate[0] << _estimate[1] << _estimate[2];
    return os.good();
}