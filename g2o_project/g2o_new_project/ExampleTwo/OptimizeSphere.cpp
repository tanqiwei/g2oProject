//
// Created by tqw on 19-1-2.
//

#include "OptimizeSphere.h"


OptimizeSphere::OptimizeSphere(std::string filename):filename{filename}
{
    // 初始化求解器
    initOptimizer();
    readG2oFile();// 读取文件
}



void OptimizeSphere::initOptimizer()
{
    // Ax = b的基本求解器，必须为不同的线性代数库重新实现
    auto linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<Block::PoseMatrixType>>();
    auto solver_ptr = g2o::make_unique<Block>(std::move(linearSolver));// 改成了使用移动构造函数构建求解器
    // 梯度下降方法：GN,LM,Dogleg中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    // GN
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    // Dogleg
    //g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(std::move(solver_ptr));
    // 图模型
    optimizer.setAlgorithm( solver );   // 设置求解器
}

void OptimizeSphere::readG2oFile()
{
    std::ifstream fin( filename );
    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    // 读取文件
    while ( !fin.eof() )
    {
        std::string name;
        fin>>name;
        if ( name == "VERTEX_SE3:QUAT" )
        {
            // SE3 顶点
            g2o::VertexSE3* v = new g2o::VertexSE3();
            int index = 0;
            fin>>index;
            v->setId( index );
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if ( index==0 )
                v->setFixed(true);
        }
        else if ( name=="EDGE_SE3:QUAT" )
        {
            // SE3-SE3 边
            g2o::EdgeSE3* e = new g2o::EdgeSE3();
            int idx1, idx2;     // 关联的两个顶点
            fin>>idx1>>idx2;
            e->setId( edgeCnt++ );
            e->setVertex( 0, optimizer.vertices()[idx1] );
            e->setVertex( 1, optimizer.vertices()[idx2] );
            e->read(fin);
            optimizer.addEdge(e);
        }
        if ( !fin.good() ) break;
    }

    std::cout<<"read total "<<vertexCnt<< " vertices, " << edgeCnt << " edges." << std::endl;

}
void OptimizeSphere::optimize()
{
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(30);// 最大迭代次数
}

void OptimizeSphere::save(std::string outputFileName)
{
    optimizer.save(outputFileName.c_str());
}
