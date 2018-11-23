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


class NewServo {
public:
    NewServo(double _lamda, double alpha, double k_0, double k_1, double k_2,
            double _period);
    ~NewServo();

    vpColVector computeControlLaw();
    vpColVector getError();
    void addFeature(vpFeaturePoint3D &s, vpFeaturePoint3D &s_star);

    void setTheta(double _theta);

private:
    std::vector<vpFeaturePoint3D *> featureList;
    std::vector<vpFeaturePoint3D *> desiredFeatureList;

    std::vector<cv::Point2d> rhoList;
    std::vector<cv::Point2d> rhoDesiredList;

    std::vector<CvPoint2D64f *> rList;
    std::vector<CvPoint2D64f *> rDesiredList;

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
