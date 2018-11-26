//
// Created by lf on 11/21/18.
//

#include <NewServo.h>
#include <math.h>

NewServo::NewServo(double _lamda, double _alpha, double k_0, double k_1, double k_2, double _period)
:lamda(_lamda), alpha(_alpha), k0(k_0), k1(k_1), k2(k_2), period(_period)
{
    initialize = true;
    iteration = 0;
}


NewServo::~NewServo() {

}
void NewServo::addFeature(vpFeaturePoint3D &s, vpFeaturePoint3D &s_star)
{
    featureList.push_back(&s);
    desiredFeatureList.push_back(&s_star);
    //std::cout <<"s: " << &s << std::endl;
    cv::Point2d rho_s, rho_star;
    rho_s.x = s.get_Y() / s.get_Z();
    rho_s.y = 1. / s.get_Z();
    rho_star.x = s_star.get_Y() / s_star.get_Z();
    rho_star.y = 1. / s_star.get_Z();

    rhoList.push_back(rho_s);
    rhoDesiredList.push_back(rho_star);
}
vpColVector NewServo::computeControlLaw() {
    vpColVector v(6);
    int n = featureList.size();
    if(initialize)
    {
        e.resize(3*n);
        start_time = std::chrono::system_clock::now();
        initialize = false;
    }

    end_time = std::chrono::system_clock::now();
    //time_elapse = (end_time - start_time).count(); // seconds
    time_elapse = period * iteration;

    for (int i = 0; i < n; ++i) {
        //std::cout <<"Z: " << featureList[i] << std::endl;
        rhoList[i].x = featureList[i]->get_Y() / featureList[i]->get_Z();
        rhoList[i].y = 1. / featureList[i]->get_Z();

        e[3*i] = lamda / (k0 - alpha) * std::exp(-alpha * time_elapse)
                + (theta0 - lamda / (k0 - alpha)) * std::exp(-k0 * time_elapse);
        e[3*i + 1] = rhoList[i].x - rhoDesiredList[i].x * std::cos(theta)
                - rhoDesiredList[i].y * std::sin(theta);
        e[3*i + 2] = rhoList[i].y + rhoDesiredList[i].x * std::sin(theta)
                - rhoDesiredList[i].y * std::cos(theta);
    }

    for (int i = 0; i < n; ++i) {
        v[5] += k0 * theta0 - lamda * std::exp(-alpha * time_elapse);
    }
    v[5] /= n;

    for (int i = 0; i < n; ++i) {
        double r1, r2;
        r1 = e[3 * i + 2];
        r2 = e[3 * i + 1] * std::exp(alpha * time_elapse);
        v[0] += k1 * r1 + k2 * r2;
        //std::cout << "r1: " << r1 << "r2: " << r2 << std::endl;
        //std::cout << "e[3 * i + 1]: " << e[3 * i + 1] << "e[3 * i + 2]: " << e[3 * i + 2] << std::endl;
    }
    v[0] /= n;
    //std::cout << "v[0]: " << n << std::endl;
    return v;
}

void NewServo::setTheta(double _theta)
{
    if(initialize) {
        theta0 = _theta;
        theta = theta0;
    }
    else
        theta = _theta;
}

vpColVector NewServo::getError()
{
    return e;
}

