//
// Created by tqw on 18-12-28.
//

#ifndef EXAMPLETWO_CREATESPHERE_H
#define EXAMPLETWO_CREATESPHERE_H

#include "commonBased.h"

class createSphere
{
public:


    /**
     * 空构造函数
     */
    createSphere();

    /**
     * 创建球体
     * @param ndsPL
     * @param lap
     * @param r
     * @param tN
     * @param rN
     */
    createSphere(std::string filename,int ndsPL,int lap,int r,Eigen::Matrix3d tN,Eigen::Matrix3d rN,bool isR);
    /**
     * 创建球体
     */
    void create();
    /**
     * 保存球体
     */
    void save();
    /**
     * GET和SET
     */

    void setFilename(std::string fileN);
    std::string getFilename();

    void setNodesPerLevel(int sN);

    int getNodesPerLevel();

    void setLaps(int lap);

    int getLaps();

    void setRadius(int r);

    int getRadius();

    void setTransNoise(Eigen::Matrix3d tN);

    Eigen::Matrix3d getTransNoise();

    void setRotNoise(Eigen::Matrix3d rN);

    Eigen::Matrix3d getRotNoise();

    void setIsRamdom(bool isR);

    bool getIsRamdom();

protected:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string filename;
    /**
     * 球上每圈有多少个节点
     */
    int nodesPerLevel;
    /**
     * 机器人在球体周围移动的次数
     */
    int laps;
    /**
     * 该球的半径
     */
    int radius;
    /**
     * 平移噪声
     */
    Eigen::Matrix3d transNoise;
    /**
     * 旋转噪声
     */
    Eigen::Matrix3d rotNoise;

    /**
     * 信息矩阵
     */
    Eigen::Matrix<double, 6, 6> information;

    /**
     *　是否使用随机种子生成球
     */
    bool isRandom;// g2o的新版本提供GaussianSampler提供seed函数

    //　生成的球体的边和顶点
    std::vector<g2o::VertexSE3*> vertices;
    std::vector<g2o::EdgeSE3*> odometryEdges;
    std::vector<g2o::EdgeSE3*> edges;


};




#endif //EXAMPLETWO_CREATESPHERE_H
