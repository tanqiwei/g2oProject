/**
 * 本次例子是优化一个
 *
 *
 *
 *
 */
#include "config.h"
#include "createSphere.h"
#include "OptimizeSphere.h"
int main(int argc,char ** argv)
{

    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\tExampleTwo:sphereCreateAndOptimize" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t anthor:YuYunTan" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t read config" << std::endl;
    if(argc != 2)
    {
        std::cout<<"Usage:ExampleTwo ../config.yaml"<<std::endl;
        return 1;
    }
    // 我们认为读取的config文件是在项目目录下
    Config::setParameterFile(argv[1]);
    std::string filename  = Config::get<std::string>("sphere.filename");
    int nodesPerLevel = Config::get<int>("sphere.nodesPerLevel");
    int laps = Config::get<int>("sphere.laps");
    int radius = Config::get<int>("sphere.radius");
    double  t1,t2,t3,r1,r2,r3;
    t1 = Config::get<double>("noiseTranslation.x");
    t2 = Config::get<double>("noiseTranslation.y");
    t3 = Config::get<double>("noiseTranslation.z");
    Eigen::Matrix3d transNoise;
    transNoise << std::pow(t1, 2),0,0,0,std::pow(t2, 2),0,0,0,std::pow(t3, 2);
    r1 = Config::get<double>("noiseRotation.x");
    r2 = Config::get<double>("noiseRotation.y");
    r3 = Config::get<double>("noiseRotation.z");
    Eigen::Matrix3d rotNoise;
    rotNoise << std::pow(r1, 2),0,0,0,std::pow(r2, 2),0,0,0,std::pow(r3, 2);
    bool isRamdomSeed = Config::get<int>("sphere.randomSeed")==0? false:true;
    std::cout << "\t\t\t\t read down" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::chrono::steady_clock::time_point time1 = std::chrono::steady_clock::now();
    std::cout << "\t\t\t\t create sphere" << std::endl;
    createSphere sphere(filename,nodesPerLevel,laps,radius,transNoise,rotNoise, isRamdomSeed);
    sphere.create();
    std::chrono::steady_clock::time_point time2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> (time2-time1);
    std::cout << "\t\t create sphere time cost = "<< time_used.count() << "seconds" <<std::endl;
    std::cout << "\t\t\t\t create down" << std::endl;
    // 保存
    sphere.save();
    std::cout << "\t\t\t\t save down" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t g2o_viewer sphere" << std::endl;
    system(("g2o_viewer "+sphere.getFilename()).c_str());
    /*
    std::string Key;
    std::cout << "\t\t\t\t Please Enter you choice:(C or E):\n" << std::endl;
    while(std::cin >> Key)
    {
        if(Key == "C")
        {
            std::cout << "\t\t\t\t will Optimize this sphere" << std::endl;
            break;
        }
        if(Key == "E")
        {
            std::cout << "\t\t\t\t will Exit this program." << std::endl;
            return 0;
        }
    }*/
    std::cout << "will Optimize this sphere,please wait 5s" << std::endl;
    sleep(5);// 间隔5s
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t ref:gaoxiang" << std::endl;
    std::cout << "====================================================" << std::endl;
    OptimizeSphere optimizeShere{filename};
    std::cout << "\t\t\t\t Optimize this sphere" << std::endl;
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    optimizeShere.optimize();
    std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
    std::cout << "\t\t\t\t End Optimize this sphere" << std::endl;
    std::chrono::duration<double> time_used1 = std::chrono::duration_cast<std::chrono::duration<double>> (t6-t5);
    std::cout << "\t\t Optimize sphere time cost = "<< time_used1.count() << "seconds" <<std::endl;
    std::cout << "\t\t\t\t Save this Optimized sphere" << std::endl;
    std::string outputFilename  = Config::get<std::string>("sphere.optimizeFilename");
    std::cout << "\t\t\t\t Save down" << std::endl;
    optimizeShere.save(outputFilename);
    std::cout << "====================================================" << std::endl;
    std::cout << "\t\t\t\t Cat this Optimized sphere" << std::endl;
    std::cout << "====================================================" << std::endl;
    system(("g2o_viewer "+outputFilename).c_str());
    return 0;
}