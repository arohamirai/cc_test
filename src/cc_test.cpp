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


using namespace std;



int main(int argc, char **argv)
{
    vpPoint oP[3], cP[3], cdP[3];
    oP[0].setWorldCoordinates(1., 0., 0.2);
    oP[1].setWorldCoordinates(4.2, 0.5, 0.1);
    oP[2].setWorldCoordinates(4.3, 0.9, 0.1);


    vpFeaturePoint3D s[3];
    vpFeaturePoint3D s_star[3];

    vpHomogeneousMatrix wMc(-4, 0.1, -0.1, 0, M_PI / 2, 0), cMw;
    vpHomogeneousMatrix wdMc(-2 ,0, 0, 0, M_PI / 2, 0);
    vpHomogeneousMatrix cdMw = wdMc.inverse();
    vpHomogeneousMatrix cdMc;

    vpSimulatorCamera robot;
    double period = 0.1;
    robot.setSamplingTime(period);

    cMw = wMc.inverse();
    for (int (i) = 0; (i) < 3; ++(i)) {
        oP[i].track(cMw);
        oP[i].print();
        cout << endl;
        s[i].buildFrom(oP[i].get_X(),  oP[i].get_Y(), oP[i].get_Z());
        std::cout << "s: " << s[i].get_X() << "  " << s[i].get_Y() << "  " << s[i].get_Z() << std::endl;

    }
    for (int (i) = 0; (i) < 3; ++(i)) {
        oP[i].track(cdMw);
        //cdP[i] = cdMw * oP[i];
        s_star[i].buildFrom(oP[i].get_X(),  oP[i].get_Y(), oP[i].get_Z());
        std::cout << "s_star: " << s_star[i].get_X() << "  " << s_star[i].get_Y() << "  " << s_star[i].get_Z() << std::endl;
    }

    NewServo task(-0.10, 0.07, 0.13, 0.0995, 0.1050, period);
    task.addFeature(s[0], s_star[0]);
    //task.addFeature(s[1], s_star[1]);
    //task.addFeature(s[2], s_star[2]);

    // TODO: CAL theta
    cdMc = cdMw * wMc;
    double theta =  std::atan2(cdMc[1][0], cdMc[0][0]);
    task.setTheta( theta);

    vpPlot graph(2, 800, 500, 400, 10, "Curves...");
    graph.initGraph(0, 2); // v. w
    graph.initGraph(1, 3); // error
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");

    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "wz");
    graph.setLegend(1, 0, "e0");
    graph.setLegend(1, 1, "e1");
    graph.setLegend(1, 2, "e2");

    vpColVector v(2);
    int n = 0;
    // cMo
    while(true)
    {
        robot.getPosition(wMc);
        cMw = wMc.inverse();
        for (int (i) = 0; (i) < 3; ++(i)) {
            oP[i].track(cMw);
            oP[i].print();
            cout << endl;
            s[i].buildFrom(oP[i].get_X(),  oP[i].get_Y(), oP[i].get_Z());
            std::cout << "s: " << s[i].get_X() << "  " << s[i].get_Y() << "  " << s[i].get_Z() << std::endl;
        }
        // TODO: CAL theta
        cdMc = cdMw * wMc;
        double theta =  std::atan2(cdMc[1][0], cdMc[0][0]);
        //std::cout <<"theta: " << theta << std::endl;
        task.setTheta( theta);

        vpColVector v_sixdof;
        v_sixdof = task.computeControlLaw();
        robot.setVelocity(vpRobot::CAMERA_FRAME, v_sixdof);
        v[0] = v_sixdof[0];
        v[1] = v_sixdof[5];

        //std::cout << "v: " << v[0] <<"  " << v[1] << endl;


        graph.plot(0, n, v);
        graph.plot(1, n, task.getError()); // plot error vector

        vpColVector error = task.getError();
        if (error[0] < 0.0001) {
            std::cout << "Reached a small error. We stop the loop... " << std::endl;
            break;
        }
        n++;
    }
    getchar();
    return 0;
}
