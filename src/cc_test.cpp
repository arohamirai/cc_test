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

void display_trajectory(const vpImage<unsigned char> &I, std::vector<Eigen::Vector2d> &point,
                        const vpCameraParameters &cam)
{
    static std::vector<vpImagePoint> traj[3];
    vpImagePoint cog;
    for (unsigned int i = 0; i < 3; i++) {
        vpMeterPixelConversion::convertPoint(cam, point[i][0], point[i][1], cog);
        traj[i].push_back(cog);
        cout << "i: " << cog.get_u() << "  " << cog.get_v() << endl;
    }
    for (unsigned int i = 0; i < 3; i++) {
        for (unsigned int j = 1; j < traj[i].size(); j++) {
            vpDisplay::displayLine(I, traj[i][j - 1], traj[i][j], vpColor::green);
        }
    }
}
void display_trajectory_desired(const vpImage<unsigned char> &I, std::vector<Eigen::Vector2d> &point,
                        const vpCameraParameters &cam)
{
    static std::vector<vpImagePoint> traj;
    vpImagePoint cog;
    for (unsigned int i = 0; i < 3; i++) {
        vpMeterPixelConversion::convertPoint(cam, point[i][0], point[i][1], cog);
        traj.push_back(cog);

        cout << "desired i: " << cog.get_u() << "  " << cog.get_v() << endl;
    }
    for (unsigned int i = 0; i < 3; i++) {
            vpDisplay::displayCircle(I, traj[i], 3, vpColor::red);
    }
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
    vpPlot graph(2, 800, 500, 400, 10, "Curves...");
    graph.initGraph(0, 2); // v. w
    graph.initGraph(1, 3*n_features); // error
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");

    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "wz");

    for (int i = 0; i < n_features; ++i) {
        graph.setLegend(1, 3*i, string("e0_" + to_string(i)));
        graph.setLegend(1, 3*i+1, string("e1_" + to_string(i)));
        graph.setLegend(1, 3*i+2, string("e2_" + to_string(i)));
    }

    vpImage<unsigned char> Iint(960, 1280, 255);
    vpDisplayOpenCV displayInt(Iint, 0, 0, "Internal view");
    vpCameraParameters cam(640, 640, Iint.getWidth() / 2, Iint.getHeight() / 2);

    int n = 0;
    vpColVector error;
    vpColVector v(2);
    while(true)
    {
        robot.getPosition(wMc_visp);
        wMc = Visp2Eigen(wMc_visp);

        // update features
        for (int i = 0; i < n_features; ++i) {
            cp[i] = wMc.inverse() * wp[i];
            cp[i] /= cp[i][0];
            pixel_cp[i] = Eigen::Vector2d(cp[i][1], cp[i][2]);
        }
        vpDisplay::display(Iint);
        display_trajectory(Iint, pixel_cp, cam);
        display_trajectory_desired(Iint, pixel_cdp, cam);
        vpDisplay::flush(Iint);

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

        // TODO: plot graph
        v[0] = v_sixdof[0];   // vx
        v[1] = v_sixdof[5];   // wz

        graph.plot(0, n, v);
        graph.plot(1, n, error);

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
