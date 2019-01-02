//
// Created by tqw on 18-12-28.
//
#include "createSphere.h"

createSphere::createSphere() {}

createSphere::createSphere(std::string filename, int ndsPL, int lap, int r, Eigen::Matrix3d tN, Eigen::Matrix3d rN,bool isR):
filename{filename},nodesPerLevel{ndsPL},laps{lap},radius{r},transNoise{tN},rotNoise{rN},isRandom{isR}
{
    information = Eigen::Matrix<double, 6, 6>::Zero();
    information.block<3,3>(0,0) = transNoise.inverse();
    information.block<3,3>(3,3) = rotNoise.inverse();
    vertices.reserve(laps*nodesPerLevel);
    edges.reserve(vertices.size());

}

void createSphere::setFilename(std::string fileN) {filename = fileN;}
std::string createSphere::getFilename() { return filename;}
void createSphere::setNodesPerLevel(int sN) {nodesPerLevel = sN;}
int createSphere::getNodesPerLevel() { return nodesPerLevel;}
void createSphere::setLaps(int lap) {laps = lap;}
int createSphere::getLaps() { return laps;}
void createSphere::setRadius(int r) {radius = r;}
int createSphere::getRadius() { return radius;}
void createSphere::setTransNoise(Eigen::Matrix3d tN) {transNoise = tN;}
Eigen::Matrix3d createSphere::getTransNoise() { return transNoise;}
void createSphere::setRotNoise(Eigen::Matrix3d rN) {rotNoise = rN;}
Eigen::Matrix3d createSphere::getRotNoise() { return rotNoise;}
void createSphere::setIsRamdom(bool isR) {isRandom = isR;}
bool createSphere::getIsRamdom() { return isRandom;}

void createSphere::create()
{
    int id = 0;
    /**
     * 生成顶点
     */
    for (int f = 0; f < laps; ++f){
        for (int n = 0; n < nodesPerLevel; ++n) {
            g2o::VertexSE3* v = new g2o::VertexSE3;
            v->setId(id++);
            Eigen::AngleAxisd rotz(-M_PI + 2*n*M_PI / nodesPerLevel, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd roty(-0.5*M_PI + id*M_PI / (laps * nodesPerLevel), Eigen::Vector3d::UnitY());
            Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();
            Eigen::Isometry3d t;
            t = rot;
            t.translation() = t.linear() * Eigen::Vector3d(radius, 0, 0);
            v->setEstimate(t);
            vertices.push_back(v);
        }
    }

    // 生成视觉里程计的边
    for (size_t i = 1; i < vertices.size(); ++i) {
        g2o::VertexSE3 *prev = vertices[i - 1];
        g2o::VertexSE3 *cur = vertices[i];
        Eigen::Isometry3d t = prev->estimate().inverse() * cur->estimate();
        g2o::EdgeSE3 *e = new g2o::EdgeSE3;
        e->setVertex(0, prev);
        e->setVertex(1, cur);
        e->setMeasurement(t);
        e->setInformation(information);
        odometryEdges.push_back(e);
        edges.push_back(e);
    }

    // 生成回环边
    for (int f = 1; f < laps; ++f) {
        for (int nn = 0; nn < nodesPerLevel; ++nn) {
            g2o::VertexSE3* from = vertices[(f-1)*nodesPerLevel + nn];
            for (int n = -1; n <= 1; ++n) {
                if (f == laps-1 && n == 1)
                    continue;
                g2o::VertexSE3* to   = vertices[f*nodesPerLevel + nn + n];
                Eigen::Isometry3d t = from->estimate().inverse() * to->estimate();
                g2o::EdgeSE3* e = new g2o::EdgeSE3;
                e->setVertex(0, from);
                e->setVertex(1, to);
                e->setMeasurement(t);
                e->setInformation(information);
                edges.push_back(e);
            }
        }
    }

    g2o::GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> transSampler;
    transSampler.setDistribution(transNoise);
    g2o::GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> rotSampler;
    rotSampler.setDistribution(rotNoise);

    //　对所有边均加噪声
    for (size_t i = 0; i < edges.size(); ++i) {
        g2o::EdgeSE3* e = edges[i];
        Eigen::Quaterniond gtQuat = (Eigen::Quaterniond) e->measurement().linear();
        Eigen::Vector3d gtTrans = e->measurement().translation();

        Eigen::Vector3d quatXYZ = rotSampler.generateSample();
        double qw = 1.0 - quatXYZ.norm();
        if (qw < 0) {
            qw = 0.;
            std::cerr << "x";
        }
        Eigen::Quaterniond rot(qw, quatXYZ.x(), quatXYZ.y(), quatXYZ.z());
        rot.normalize();
        Eigen::Vector3d trans = transSampler.generateSample();
        rot = gtQuat * rot;
        trans = gtTrans + trans;
        Eigen::Isometry3d noisyMeasurement = (Eigen::Isometry3d) rot;
        noisyMeasurement.translation() = trans;
        e->setMeasurement(noisyMeasurement);
    }

    // 连接所有的里程计算约束来计算初始状态
    for (size_t i =0; i < odometryEdges.size(); ++i) {
        g2o::EdgeSE3* e = edges[i];
        g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(e->vertex(0));
        g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(e->vertex(1));
        g2o::HyperGraph::VertexSet aux; aux.insert(from);
        e->initialEstimate(aux, to);
    }

}

void createSphere::save()
{
    //文件输入
    std::ofstream fileOutputStream;
    if (filename != "-") {
        std::cerr << "Writing into " << filename << std::endl;
        fileOutputStream.open(filename.c_str());
    } else {
        std::cerr << "writing to stdout" << std::endl;
    }

    std::string vertexTag = g2o::Factory::instance()->tag(vertices[0]);
    std::string edgeTag = g2o::Factory::instance()->tag(edges[0]);

    std::ostream& fout = filename != "-"?fileOutputStream:std::cout;
    for (size_t i = 0; i < vertices.size(); ++i) {
        g2o::VertexSE3* v = vertices[i];
        fout << vertexTag << " " << v->id() << " ";
        v->write(fout);
        fout << std::endl;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        g2o::EdgeSE3* e = edges[i];
        g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(e->vertex(0));
        g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(e->vertex(1));
        fout << edgeTag << " " << from->id() << " " << to->id() << " ";
        e->write(fout);
        fout << std::endl;
    }

}