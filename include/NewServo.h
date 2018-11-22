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

    // oP to image plane
    vpPoint world2image(const vpPoint &oP, const vpMatrix &intrinsic);

    // image plane to normalized plane
    vpPoint image2normalized(const vpPoint &iP);

    //world2normalized

    void addFeature(cv::Point3d &s, cv::Point3d &s_star);

    double setTheta(double );

private:
    std::list<cv::Point3d *> featureList;
    std::list<cv::Point3d *> desiredFeatureList;
    std::list<unsigned int> featureSelectionList;

    std::list<CvPoint2D64f> rhoList;
    std::list<CvPoint2D64f> rhoDesiredList;
    std::list<unsigned int> rhoFeatureSelectionList;

    std::list<CvPoint2D64f *> rList;
    std::list<CvPoint2D64f *> rDesiredList;
    std::list<unsigned int> rFeatureSelectionList;

    std::list<double> error;

    double lamda;
    double alpha;
    double k0;
    double k1;
    double k2;
    double period;
    std::chrono::system_clock::time_point start_time;
    std::chrono::system_clock::time_point end_time;
    std::chrono::duration<double> time_elapse;

    bool initialize;
    double theta0;
    vpColVector v;

    int iteration;

    vpHomogeneousMatrix cMo;
};


#endif //CC_TEST_NEWSERVO_H
