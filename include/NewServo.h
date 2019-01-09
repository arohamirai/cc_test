//
// Created by lf on 11/21/18.
//

#ifndef CC_TEST_NEWSERVO_H
#define CC_TEST_NEWSERVO_H
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpBasicFeature.h>
#include <list>
#include "opencv2/opencv.hpp"
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>


class NewServo {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum {
        SELECT_XY, SELECT_XZ, SELECT_ZX, SELECT_ZY
    };
    NewServo(double _lamda, double alpha, double k_0, double k_1, double k_2,
            double _period);
    ~NewServo();

    vpColVector computeControlLaw();
    vpColVector getError();
    void addFeature( Eigen::Vector3d &s,  Eigen::Vector3d &s_star, const unsigned int select);

    void setTheta(double _theta);

private:
    std::vector< Eigen::Vector3d * > featureList;
    std::vector< Eigen::Vector3d * > desiredFeatureList;

    int select_type_;

    std::vector<Eigen::Vector2d> rhoList;
    std::vector<Eigen::Vector2d> rhoDesiredList;

    std::vector<Eigen::Vector2d> rList;
    std::vector<Eigen::Vector2d> rDesiredList;

    std::list<double> error;

    double lamda;
    double alpha;
    double k0;
    double k1;
    double k2;
    double period;
    std::chrono::system_clock::time_point start_time;
    std::chrono::system_clock::time_point end_time;
    //std::chrono::duration<double> time_elapse;
    double time_elapse; // seconds;

    bool initialize;
    double theta0;
    double theta;
    vpColVector e;

    int iteration;
};


#endif //CC_TEST_NEWSERVO_H
