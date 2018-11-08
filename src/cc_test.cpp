#include "boost/variant.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/make_shared.hpp>
#include <boost/timer/timer.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <jsoncpp/json/json.h>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/dir.h>
#include <sys/errno.h>
#include <time.h>
#include <mutex>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
/*
 * 问题引出：有一个曲线函数 y = exp(ax^2 + bx + c), 现在有一批输入和输出点对(x_i, y_i)，由于测量误差和噪声的存在，
 * 需要由这些点拟合出曲线的解析方程，即求出a,b,c使得整体误差最小
 */

//曲线模型顶点，模板参数：优化变量的维度和数据类型
class CurveFittingVertex:public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void setToOriginImpl()    // 重置
  {
    _estimate << 0, 0, 0;
  }

  virtual void oplusImpl(const double* update) // 更新, 增量方程(x_new = x_old + delta_x)
  {
    _estimate += Eigen::Vector3d(update);
  }

  // 存盘和读盘： 留空
  virtual bool read(istream &is) {}
  virtual bool write(ostream &os) const {}
};

// 误差模型边，模板参数： 观测值维度， 类型， 连接顶点类型
class CurveFittingEdge:public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(double x): BaseUnaryEdge(), _x(x) {}

  // 计算曲线模型边的误差
  void computeError()
  {
    const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);             //_vertices 属于Edge的成员
    const Eigen::Vector3d abc = v->estimate();                                                      //_estimate 属于Vertex的成员
    _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));       //_measurement，_error属于Edge的成员，_measurement为字面量，_error为列向量
  }

  virtual bool read(istream &is) {}
  virtual bool write(ostream &os) const {}
public:
  double _x; // x值， y值为_measurement
};

int main(int argc, char **argv)
{
  double a = 1.0, b = 2.0, c = 1.0; // 真实参数值
  int N = 100;                      // 数据点个数(样本点)
  double w_sigma = 1.0;             // 噪声Sigma值
  cv::RNG rng;                      // OpenCV随机数产生器
  double abc[3] = {0, 0, 0};        // abc参数的估计值（初始值）

  vector<double> x_data, y_data;            // 样本数据
  cout << "generating data: " << endl;
  for(int i = 0; i <N; i++)
  {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
  }

  // 构建图优化，先设定g2o
  // 矩阵块： 每个误差项优化维度为 3, 误差值维度为 1
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;
  // 线性方程求解器： 稠密的增量方程
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(linearSolver);                    //矩阵块求解器
  // 梯度下降方法： 从GN， LM， DogLeg 中选择
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  // 图模型
  g2o::SparseOptimizer optimizer;
  // 设置求解器
  optimizer.setAlgorithm(solver);
  // 打开调试输出
  optimizer.setVerbose(true);

  // 向图中增加顶点
  CurveFittingVertex* v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(0, 0, 0));
  v->setId(0);                                //设置顶点id
  optimizer.addVertex(v);

  // 向图中增加边
  for(int i = 0; i < N; i++)
  {
    CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v);                   // 设置参数1编号的顶点指向v顶点
    edge->setMeasurement(y_data[i]);        // 设置观测数据
    // 设置信息矩阵： 协方差矩阵之逆(顶点之间误差之间的协方差组成的矩阵)
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / w_sigma * w_sigma);
    optimizer.addEdge(edge);
  }

  // 执行优化
  cout << "start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(100);                  //迭代次数
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost " << time_used.count() << " seconds. " << endl;

  // 输出优化值
  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "estimated model: " << abc_estimate.transpose() << endl;

  return 0;
}
