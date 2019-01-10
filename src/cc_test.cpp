#include <iostream>
#include <opencv2/opencv.hpp>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpSimulatorPioneer.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include "NewServo.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <opencv2/opencv.hpp>
using namespace std;


vpHomogeneousMatrix Eigen2Visp(Eigen::Affine3d& m)
{
    vpHomogeneousMatrix m_;
    m_.eye();
    for(int i = 0; i < 4; i++)
        for (int j = 0; j < 4; ++j) {
            m_[i][j] = m.matrix()(i, j);
        }
    return m_;
}

Eigen::Affine3d Visp2Eigen(vpHomogeneousMatrix& m)
{
    Eigen::Affine3d m_;
    m_.Identity();
    for(int i = 0; i < 4; i++)
        for (int j = 0; j < 4; ++j) {
            m_.matrix()(i, j) = m[i][j];
        }
    return m_;
}

int main(int argc, char **argv)
{
    double lamda = -0.10;
    double alpha = 0.07;
    double k0 = 0.13;

    double k1 = 0.0995;
    double k2 = 0.1050;
    double period = 0.1;
    double theta;

    cv::RNG rng;
    Eigen::Vector3d wp[3], cp[3], cdp[3];
    int n_features = 1;
    Eigen::Affine3d wMc, wMcd;
    Eigen::Affine3d cMcd;
    vpHomogeneousMatrix wMc_visp;

    wp[0] = Eigen::Vector3d( 0., 0.2, 4.1);
    wp[1] = Eigen::Vector3d( 0.5, 0.1, 8.2);
    wp[2] = Eigen::Vector3d( 0.9, 0.5, 8.3);

    // INIT wMc, wMcd
    wMc.setIdentity();
    //wMc.prerotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d ( 0,1,0 )));
    wMc.pretranslate(Eigen::Vector3d(0.2, 0.1, 2));

    wMcd.setIdentity();
    //wMcd.prerotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d ( 0,0,1 )));
    //wMcd.prerotate(Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d ( 0,1,0 )));
    wMcd.pretranslate(Eigen::Vector3d(0.2, 0.1, 4));

    // Init servo
    NewServo task(lamda, alpha, k0,  k1, k2, period);
    // addFeature(points in camera, normalized)
    for (int i = 0; i < n_features; ++i) {
        Eigen::Vector3d chaos = Eigen::Vector3d(rng.uniform(0., 0.01), rng.uniform(0., 0.01), rng.uniform(0., 0.01));
        cp[i] = wMc.inverse() * (wp[i] + chaos);
        cdp[i] = wMcd.inverse() * (wp[i] + chaos);
        task.addFeature(cp[i], cdp[i], NewServo::SELECT_ZX);
    }

    // init robot
    vpSimulatorCamera robot;
    robot.setSamplingTime(period);
    robot.setPosition(Eigen2Visp(wMc));

    // init graph
    vpPlot graph(4, 800, 1000);
    graph.initGraph(0, 2); // v. w
    graph.initGraph(1, 3*n_features); // error
    graph.initGraph(2, 1);   // cMo
    graph.initGraph(3, 2);   // wMc
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");
    graph.setTitle(2, "oMc");
    graph.setTitle(3, "wMc");

//    graph.initRange(2, -1, 1, -5, 5);
//    graph.initRange(3, -1, 1, -5, 5);

    graph.setLegend(0, 0, "vz");
    graph.setLegend(0, 1, "wy");
    for (int i = 0; i < n_features; ++i) {
        graph.setLegend(1, 3*i, string("e0_" + to_string(i)));
        graph.setLegend(1, 3*i+1, string("e1_" + to_string(i)));
        graph.setLegend(1, 3*i+2, string("e2_" + to_string(i)));
    }

    graph.setLegend(3, 0, "traj");
    graph.setLegend(3, 1, "heading");
    int n = 0;
    vpColVector error;
    vpColVector v(2);
    while(true)
    {
        robot.getPosition(wMc_visp);
        wMc = Visp2Eigen(wMc_visp);

        // update features
        for (int i = 0; i < n_features; ++i) {
            Eigen::Vector3d chaos = Eigen::Vector3d(rng.uniform(0., 0.01), rng.uniform(0., 0.01), rng.uniform(0., 0.01));
            cp[i] = wMc.inverse() * (wp[i] + chaos);
        }
        // update theta
        cMcd = wMc.inverse() * wMcd;
//
//        cout << "wMcd: \n";
//        cout << wMcd.matrix()<< endl;
//        cout << "wMc: \n";
//        cout << wMc.matrix()<< endl;
//        cout << "cMcd: \n";
//        cout << cMcd.matrix()<< endl;
        //getchar();

        Eigen::Vector3d euler_angles = cMcd.rotation().eulerAngles ( 2, 1, 0);
        theta = euler_angles[1];

        task.setTheta(theta);
        cout << "theta: " << theta << endl;
        //return 0;

        // get velocity
        vpColVector vel, v_sixdof(6);
        vel = task.computeControlLaw();
        error = task.getError();
        // publish vel
        v_sixdof[2] = vel[0];
        v_sixdof[4] = vel[1];
        robot.setVelocity(vpRobot::CAMERA_FRAME, v_sixdof);

        // plot graph
        v[0] = v_sixdof[2];   // vz
        v[1] = v_sixdof[4] * 10;   // wy


        graph.plot(0, n, v);

        // plot error
        graph.plot(1, n, error);

        // cdMc
        //graph.plot(2, 0, cMcd.inverse()(0,3), cMcd.inverse()(2,3));       // x,z

        vpColVector e_theta(1);
        e_theta[0] = theta * 180 / M_PI;
        graph.plot(2, n, e_theta );

        vpColVector pz(4), pz_w(4);
        pz[0] = 0.1;
        pz[1] = 0;
        pz[2] = 0;
        pz[3] = 1;

        pz_w = Eigen2Visp(wMc) * pz;

        //cout << wMc.matrix()<<endl;
        //cout << pz_w[0] << " " << pz_w[1] << " " << pz_w[2] << " " << pz_w[3] <<endl;
        //return 0;

//        graph.plot(3, 1, wMc(0, 3), wMc(2,3));
//        graph.plot(3, 1, pz_w[0], pz_w[2]);
//        graph.plot(3, 1, wMc(0, 3), wMc(2,3));

        // wMc
        //graph.plot(3, 0, wMc.matrix()(0,3), wMc.matrix()(2,3));     // x, z

        //getchar();
        //usleep(0.1*1000000);
        // whether to stop
        if(error.sumSquare()< 0.001 && n > 1)
        {
            std::cout << "Reached a small error. We stop the loop... " << std::endl;
            std::cout << "cMcd:" << cMcd.matrix() << std::endl;
            std::cout << "wMc:" << wMc.matrix() << std::endl;
            break;
        }
//        if(n > 5000)
//            break;
        n++;
    }

    getchar();
    return 0;
}
