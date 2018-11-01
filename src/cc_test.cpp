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

//#include <visp3/visual_features/vpFeatureBuilder.h>
//#include <visp3/visual_features/vpFeatureDepth.h>
//#include <visp3/visual_features/vpFeaturePoint.h>
//#include <visp3/core/vpHomogeneousMatrix.h>
//#include <visp3/gui/vpPlot.h>
//#include <visp3/vs/vpServo.h>
//#include <visp3/robot/vpSimulatorPioneer.h>
//#include <visp3/core/vpVelocityTwistMatrix.h>

using namespace std;

int main(int argc, char **argv)
{
  boost::timer::cpu_timer t;
  t.start();
  for( int i = 0; i < 10; i++)
  {
    sleep(1);
    cout<<"time elapsed: "<< t.elapsed().wall/1e9<<endl;          // nanosecond: 1ns = 1x10-9s
    t.start();
  }
  return 0;
}
