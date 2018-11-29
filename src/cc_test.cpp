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

    Eigen::Vector3d wp[3], cp[3], cdp[3];
    std::vector<Eigen::Vector2d> pixel_cp, pixel_cdp;
    int n_features = 3;
    Eigen::Affine3d wMc, wMcd;
    Eigen::Affine3d cMcd;
    vpHomogeneousMatrix wMc_visp;

    wp[0] = Eigen::Vector3d(1., 0., 0.2);
    wp[1] = Eigen::Vector3d(4.2, 0.5, 0.3);
    wp[2] = Eigen::Vector3d(4.3, 0.9, 0.2);

    // INIT wMc, wMcd
    wMc.setIdentity();
    wMc.prerotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d ( 0,0,1 )));
    wMc.pretranslate(Eigen::Vector3d(-4, 0.2, 0.1));

    wMcd.setIdentity();
    //wMcd.prerotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d ( 0,0,1 )));
    //wMcd.prerotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d ( 1,0,0 )));
    wMcd.pretranslate(Eigen::Vector3d(-2 ,0.1, 0.1));

    // Init servo
    NewServo task(lamda, alpha, k0,  k1, k2, period);
    // addFeature(points in camera, normalized)
    for (int i = 0; i < n_features; ++i) {
        cp[i] = wMc.inverse() * wp[i];
        cp[i] /= cp[i][0];
        cdp[i] = wMcd.inverse() * wp[i];
        cdp[i] /= cdp[i][0];
        task.addFeature(cp[i], cdp[i]);

        pixel_cp.push_back(Eigen::Vector2d(cp[i][1], cp[i][2]));
        pixel_cdp.push_back(Eigen::Vector2d(cdp[i][1], cdp[i][2]));
    }

    // init robot
    vpSimulatorCamera robot;
    robot.setSamplingTime(period);
    robot.setPosition(Eigen2Visp(wMc));

    // init graph
    vpPlot graph(4, 800, 500, 400, 10, "Curves...");
    graph.initGraph(0, 2); // v. w
    graph.initGraph(1, 3*n_features); // error
    graph.initGraph(2, 1);   // cMo
    graph.initGraph(3, 1);   // wMc
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");
    graph.setTitle(2, "oMc");
    graph.setTitle(3, "wMc");

    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "wz");
    for (int i = 0; i < n_features; ++i) {
        graph.setLegend(1, 3*i, string("e0_" + to_string(i)));
        graph.setLegend(1, 3*i+1, string("e1_" + to_string(i)));
        graph.setLegend(1, 3*i+2, string("e2_" + to_string(i)));
    }

    int n = 0;
    vpColVector error;
    vpColVector v(2);
    while(true)
    {
        robot.getPosition(wMc_visp);
        wMc = Visp2Eigen(wMc_visp);
        std::cout << "wMc:" <<wMc.matrix() << endl;

        // update features
        for (int i = 0; i < n_features; ++i) {
            cp[i] = wMc.inverse() * wp[i];
            cp[i] /= cp[i][0];
            pixel_cp[i] = Eigen::Vector2d(cp[i][1], cp[i][2]);
        }
        // update theta
        cMcd = wMc.inverse() * wMcd;
        theta = std::atan2(cMcd.matrix()(1, 0), cMcd.matrix()(0, 0));
        task.setTheta(theta);
        //cout << "theta: " << theta << endl;

        // get velocity
        vpColVector v_sixdof;
        v_sixdof = task.computeControlLaw();
        error = task.getError();
        // publish vel
        robot.setVelocity(vpRobot::CAMERA_FRAME, v_sixdof);

        // plot graph
        v[0] = v_sixdof[0];   // vx
        v[1] = v_sixdof[5];   // wz

        // plot error
        graph.plot(0, n, v);
        graph.plot(1, n, error);

        // cMcd
        vpColVector lateral(1);
        lateral[0] = cMcd.inverse().matrix()(1,3);
        graph.plot(2, cMcd.inverse().matrix()(0,3), lateral);

        // wMc
        vpColVector world_y(1);
        world_y[0] = wMc.matrix()(1,3);
        graph.plot(3, wMc.matrix()(0,3), world_y);

        // whether to stop
        if(fabs(error[0])< 0.001 && n > 1)
        {
            std::cout << "Reached a small error. We stop the loop... " << std::endl;
            break;
        }
        if(n > 5000)
            break;
        n++;
    }

    getchar();
    return 0;
}
