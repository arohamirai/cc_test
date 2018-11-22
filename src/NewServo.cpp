//
// Created by lf on 11/21/18.
//

#include <NewServo.h>
#include <math.h>

NewServo::NewServo(dobule _lamda, double _alpha, double k_0, double k_1, double k_2, double _period)
:lamda(_lamda), alpha(_alpha), k0(k_0), k1(k_1), k2(k_2), period(_period)
{
    iter = 0;
    initialize = true;
    iteration = 0;
    v.resize(6);
}


NewServo::~NewServo() {

}
void NewServo::addFeature(cv::Point3d &s, cv::Point3d &s_star)
{
    featureList.push_back(&s);
    desiredFeatureList.push_back(&s_star);

    cv::Point2d rho_s, rho_star;
    rho_s.x = s.y / s.z;
    rho_s.y = 1. / s.z;
    rho_star.x = s_star. y / s_star.z;
    rho_star.y = 1. / s_star.z;

    rhoList.push_back(rho_s);
    rhoDesiredList.push_back(rho_star);
}
vpColVector NewServo::computeControlLaw() {
    int n = featureList.size();
    int r = 3 * n;
    int c = 3
    vpMatrix mat_e(3*n, 1);



    if(initialize)
    {
        theta0 = solveTheta();
        mat_e[]
        start_time = std::chrono::system_clock::now();
        initialize = false;
    }

    end_time = std::chrono::system_clock::now();
    time_elapse = (end_time - start_time).count(); // seconds

    double theta = solveTheta();
    e0 = lamda / (k0 - alpha) * std::exp(-alpha * time_elapse) + (theta0 - lamda / (k0 - alpha))
            * std::exp(-k0 * time_elapse);
    e1 =
    v[5] = k0 * theta0 - lamda * std::exp(-alpha * time_elapse);

    vpMatrix mat;
    mat.pseudoInverse()

    return vpColVector();
}

double NewServo::solveTheta()
{
    return 0.0;
}


