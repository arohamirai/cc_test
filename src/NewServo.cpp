//
// Created by lf on 11/21/18.
//

#include <NewServo.h>
#include <math.h>

NewServo::NewServo(double _lamda, double _alpha, double k_0, double k_1, double k_2, double _period)
:lamda(_lamda), alpha(_alpha), k0(k_0), k1(k_1), k2(k_2), period(_period)
{
    initialize = false;
    iteration = 0;
}


NewServo::~NewServo() {

}
void NewServo::addFeature( Eigen::Vector3d &s,  Eigen::Vector3d &s_star, const unsigned int select)
{
    featureList.push_back(&s);
    desiredFeatureList.push_back(&s_star);

    Eigen::Vector3d normalized_s, normalized_s_star ;
    Eigen::Vector2d rho_s, rho_star;

    select_type_ = select;
    if(select_type_ == SELECT_XY)   // x is the light coor , z vertical
    {
        normalized_s = s / s[0];
        normalized_s_star = s_star / s_star[0];

        rho_s[0] = normalized_s[1] / normalized_s[2];
        rho_s[1] = 1. / normalized_s[2];
        rho_star[0] = normalized_s_star[1] / normalized_s_star[2];
        rho_star[1] = 1. / normalized_s_star[2];
    }
    else if(select_type_ == SELECT_XZ)
    {
        normalized_s = s / s[0];
        normalized_s_star = s_star / s_star[0];

        rho_s[0] = normalized_s[2] / normalized_s[1];
        rho_s[1] = 1. / normalized_s[1];
        rho_star[0] = normalized_s_star[2] / normalized_s_star[1];
        rho_star[1] = 1. / normalized_s_star[1];
    }
    else if(select_type_ == SELECT_ZX)    // z is the light coor, y is vertical
    {
        normalized_s = s / s[2];
        normalized_s_star = s_star / s_star[2];

        rho_s[0] = normalized_s[0] / normalized_s[1];
        rho_s[1] = 1. / normalized_s[1];
        rho_star[0] = normalized_s_star[0] / normalized_s_star[1];
        rho_star[1] = 1. / normalized_s_star[1];
    }
    else if(select_type_ == SELECT_ZY)      // Z is forward, y x is vertical
    {
        normalized_s = s / s[2];
        normalized_s_star = s_star / s_star[2];

        rho_s[0] = normalized_s[1] / normalized_s[0];
        rho_s[1] = 1. / normalized_s[0];
        rho_star[0] = normalized_s_star[1] / normalized_s_star[0];
        rho_star[1] = 1. / normalized_s_star[0];
    }
    rhoList.push_back(rho_s);
    rhoDesiredList.push_back(rho_star);
}
vpColVector NewServo::computeControlLaw() {
    vpColVector v(2, 0);
    static int n_features = featureList.size();
    if(!initialize)
    {
        e.resize(3*n_features);
        start_time = std::chrono::system_clock::now();
        initialize = true;
        std::cout << "2222";
    }
    end_time = std::chrono::system_clock::now();
    //time_elapse = (end_time - start_time).count(); // seconds
    time_elapse = period * iteration;

    for(int i = 0; i < n_features; i++)
    {
        std::cout << "333";
        if(select_type_ == SELECT_XY)   // x is the light coor , z vertical
        {
            Eigen::Vector3d normalized_s = (*featureList[i]) / ((*featureList[i])[0]);
            rhoList[i][0] = normalized_s[1] / normalized_s[2];
            rhoList[i][1] = 1. / normalized_s[2];
        }
        else if(select_type_ == SELECT_XZ)
        {
            Eigen::Vector3d normalized_s = (*featureList[i]) / ((*featureList[i])[0]);
            rhoList[i][0] = normalized_s[2] / normalized_s[1];
            rhoList[i][1] = 1. / normalized_s[1];
        }
        else if(select_type_ == SELECT_ZX)    // z is the light coor, y is vertical
        {
            Eigen::Vector3d normalized_s = (*featureList[i]) / ((*featureList[i])[2]);
            rhoList[i][0] = normalized_s[0] / normalized_s[1];
            rhoList[i][1] = 1. / normalized_s[1];
        }
        else if(select_type_ == SELECT_ZY)      // Z is forward, y x is vertical
        {
            Eigen::Vector3d normalized_s = (*featureList[i]) / ((*featureList[i])[2]);
            rhoList[i][0] = normalized_s[1] / normalized_s[0];
            rhoList[i][1] = 1. / normalized_s[0];
        }
    }

    e[0] = theta;

    e[1] = (rhoList[0][0] - rhoDesiredList[0][0] * std::cos(theta)
            - rhoDesiredList[0][1] * std::sin(theta));
    e[2] = (rhoList[0][1] + rhoDesiredList[0][0] * std::sin(theta)
            - rhoDesiredList[0][1] * std::cos(theta));

    /*
    std::cout<< "rhoList: "<< rhoList[i][0] << "  " << rhoList[i][1]
    <<" ||| " << rhoDesiredList[i][0] << "  " <<rhoDesiredList[i][1] << std::endl;
    std::cout<< "e: "<< e[3*i ] << "  " << e[3*i + 1]
    <<"  " << e[3*i + 2] << "  "  << std::endl;
    /**/
    double r1, r2;
    r1 = e[2];
    r2 = e[1] * std::exp(alpha * time_elapse);
    v[0] = k1 * r1 + k2 * r2;               // v

    v[1] = k0 * theta - lamda * std::exp(-alpha * time_elapse);     //w

    iteration++;
    return v;
}

void NewServo::setTheta(double _theta)
{
    if(!initialize) {
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

