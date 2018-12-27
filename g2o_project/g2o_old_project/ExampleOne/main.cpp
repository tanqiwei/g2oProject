/**
 * 这是关于旧版本即SLAM十四讲的g2o版本进行的代码编写和注释
 * 旧版本和新版本在用法上不一定一样
 */



#include "commonBased.h"
#include "CurveFittingVetex.h"
#include "CurveFittingEdge.h"



int main()
{
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t这是关于曲线拟合的例子" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t anthor:YuYunTan" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\treference:gaoxiang" << std::endl;
    std::cout << "====================================================" << std::endl;

    // 真实参数,假设曲线是f(x)=a*x^2+b*x+c,真实情况下应该是f(x)=x^2+2*x+1
    double a = 1.0,b=2.0,c =1.0;
    int N = 100;// 数据点的数量
    double w_sigma = 1.0;// 噪声的方差Sigma
    //cv::RNG rng;// opencv随机数产生器,如果要使用opencv的高斯0均值随机数发生器,请在commonBased引入[将注释取消即可]
    double abc[3] = {0,0,0};// abc参数的估计值

    std::vector<double> x_data,y_data;//保存随机数的值,即模拟真实点的情况
    std::cout << "\t\t\t\t generating data:" << std::endl;
    std::cout << "====================================================" << std::endl;
    for(auto i=0;i<N;i++)
    {
        double  x = i/100.0;
        x_data.push_back(x);
        y_data.push_back(
          //exp(a*x*x+b*x+c)+rng.gaussian(w_sigma)
          exp(a*pow(x,2)+b*x+c)+g2o::Sampler::gaussRand(0,w_sigma)// 采用G2O的随机发生器
        );

        std::cout << x_data[i] << " " << y_data[i] << std::endl;
    }
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t       构建图:" << std::endl;
    // 构建g2o图优化问题

    // 矩阵块: 每个误差项优化维度为3，误差值维度为1
    /**
     * 要优化的变量是(a,b,c),这个变量含有三个维度,在eigen3中最好的存储方式是Eigen:Vector3d
     * 误差是一维的,因为结果值是一维度的.两者相减还是一个维度
     */
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(linearSolver);
    // 梯度下降方法：GN,LM,Dogleg中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    // GN
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    // Dogleg
    //g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
    // 图模型
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);// 设置求解器
    optimizer.setVerbose(true);// 打开调试输出

    // 对图中添加顶点
    CurveFittingVetex* v = new CurveFittingVetex();
    v->setEstimate(Eigen::Vector3d(0,0,0));
    v->setId(0);
    optimizer.addVertex(v);
    // 全程只有一个顶点,该顶点用多条一元边连接
    for(int i=0;i<N;i++)
    {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);// 边的编号
        edge->setVertex(0,v);//设置超边的第几个顶点是什么顶点,目前只有一个顶点
        edge->setMeasurement(y_data[i]);//设置观测值
        edge->setInformation(Eigen::Matrix<double ,1,1>::Identity()*1/(w_sigma*w_sigma));//信息矩阵是协方差的逆,1/sigma^2
        optimizer.addEdge(edge);// 图中加入顶点

    }
    std::cout << "\t\t\t\t     构建完成" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t start optimization" << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);//设置最大迭代次数
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> (t2-t1);
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t solve time cost = "<< time_used.count() << "seconds" <<std::endl;
    std::cout << "====================================================" << std::endl;
    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    std::cout << "\t\t estimated model:" << abc_estimate.transpose() << std::endl;

    return 0;
}