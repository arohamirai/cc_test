#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/clamp.hpp>

using namespace std;

// 代价函数计算模型
struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y){}
  //残差的计算
  template <typename T>
  bool operator ()  (
      const T* const abc,             // 模型参数，有3维
      T* residual ) const             //残差
  {
    // y - exp(ax^2 + bx + c)
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
    return true;
  }

  const double _x, _y;                // x, y数据
};

int main(int argc, char **argv)
{

  int foo = 5;
  foo = boost::algorithm::clamp ( foo, 1, 10 );
  cout << "foo: " << foo<< endl;
    char a[7];

    cout << "finish" << endl;
  return 0;
}
