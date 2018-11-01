#include "boost/variant.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/make_shared.hpp>
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

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorPioneer.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

using namespace std;

int main(int argc, char **argv)
{

    vpHomogeneousMatrix wMe;
    wMe.buildFrom(100,200,300, M_PI/2,M_PI/3,M_PI/4);

    cout<<"wMe:";
    wMe.print();
    cout<<endl;

    double yaw;
    vpThetaUVector euler_angle;
    wMe.extract(euler_angle);

    cout<<"euler:";
    cout<<euler_angle[0] << "  "<<euler_angle[1] << "  "<<euler_angle[2] << "  "<<endl;

    return 0;
}
