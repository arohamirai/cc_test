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

    vpHomogeneousMatrix wMc, cMw;
    vpTranslationVector wtc;  // mm
    vpRzyxVector wrc;         // radian
    wtc.buildFrom(-4, 0.1, -0.1);
    wrc.buildFrom(0, M_PI / 2, 0);
    wMc.buildFrom(wtc, vpRotationMatrix(wrc));

    vpHomogeneousMatrix wdMc;
    vpTranslationVector wdtc;  // mm
    vpRzyxVector wdrc;         // radian
    wdtc.buildFrom(-2 ,0, 0);
    wdrc.buildFrom(0, M_PI / 2, M_PI / 4);
    wdMc.buildFrom(wdtc, vpRotationMatrix(wdrc));


    vpHomogeneousMatrix cdMw = wdMc.inverse();
    vpHomogeneousMatrix cdMc, cMdc;

    vpSimulatorCamera robot;
    double period = 0.1;
    robot.setSamplingTime(period);
    robot.setPosition(wMc);

    cMw = wMc.inverse();
    for (int (i) = 0; (i) < 3; ++(i)) {
        oP[i].track(cMw);
        //oP[i].print();
        //cout << endl;
        s[i].buildFrom(oP[i].get_X(),  oP[i].get_Y(), oP[i].get_Z());
       // vpHomogeneousMatrix::saveYAML("cMw.dat", cMw);
        //std::cout << "s: " << s[i].get_X() << "  " << s[i].get_Y() << "  " << s[i].get_Z() << std::endl;
    }
    for (int (i) = 0; (i) < 3; ++(i)) {
        oP[i].track(cdMw);
        //cdP[i] = cdMw * oP[i];
        s_star[i].buildFrom(oP[i].get_X(),  oP[i].get_Y(), oP[i].get_Z());
        //std::cout << "s_star: " << s_star[i].get_X() << "  " << s_star[i].get_Y() << "  " << s_star[i].get_Z() << std::endl;
    }

    NewServo task(-0.10, 0.07, 0.13, 0.0995, 0.1050, period);
    task.addFeature(s[0], s_star[0]);
    //task.addFeature(s[1], s_star[1]);
    //task.addFeature(s[2], s_star[2]);

    // TODO: CAL theta
    cdMc = cdMw * wMc;
    //cdMc.print();
    //cout << endl;
    cMdc = cdMc.inverse();
    double theta =  std::atan2(cMdc[2][1], cMdc[2][2]);  // rot x
    cout <<"Init Theta: " << theta << endl;
    task.setTheta(theta);
    cout << " Init cMdc: ";
    cMdc.print();
    cout << endl;


    vpPlot graph(3, 800, 500, 400, 10, "Curves...");
    graph.initGraph(0, 2); // v. w
    graph.initGraph(1, 3); // error
    graph.initGraph(2, 1); // traj
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");
    graph.setTitle(2, "traj");

    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "wz");
    graph.setLegend(1, 0, "e0");
    graph.setLegend(1, 1, "e1");
    graph.setLegend(1, 2, "e2");
    graph.setLegend(2, 0, "traj");

    vpColVector v(2);
    int n = 0;
    // cMo
    while(true)
    {
        robot.getPosition(wMc);
        cMw = wMc.inverse();
        for (int (i) = 0; (i) < 3; ++(i)) {
            oP[i].track(cMw);
            //oP[i].print();
            //cout << endl;
            s[i].buildFrom(oP[i].get_X(),  oP[i].get_Y(), oP[i].get_Z());
            //std::cout << "s: " << s[i].get_X() << "  " << s[i].get_Y() << "  " << s[i].get_Z() << std::endl;
        }
        // TODO: CAL theta
        cdMc = cdMw * wMc;
        cMdc = cdMc.inverse();
        theta =  std::atan2(cMdc[2][1], cMdc[2][2]);  // rot x
        //std::cout <<"theta: " << theta << std::endl;
        task.setTheta( theta);
        //cMdc.print();
        //cout << endl;
        vpColVector v_sixdof;
        v_sixdof = task.computeControlLaw();
        robot.setVelocity(vpRobot::CAMERA_FRAME, v_sixdof);
        v[0] = v_sixdof[0];
        v[1] = v_sixdof[3];

        std::cout <<"theta: " << theta << " v[1]:" << v[1] << std::endl;

        //std::cout << "v: " << v[0] <<"  " << v[1] << endl;

        graph.plot(0, n, v);
        graph.plot(1, n, task.getError()); // plot error vector

        vpColVector traj(1);
        traj[0] = cdMc[2][3];
        graph.plot(2, n, traj);
        //cout << "traj:" << cdMc[2][3] << endl;

        vpColVector error = task.getError();
        vpColVector::saveYAML("e.dat", error);
        if (fabs(error[0]) < 0.1) {
            std::cout << "Reached a small error. We stop the loop... " << std::endl;
            cout << "n: " << n << endl;
            break;
        }
        if(n > 200)
            break;
        n++;
    }
    getchar();
    return 0;
}
