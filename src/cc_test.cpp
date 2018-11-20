#include <iostream>
#include <opencv2/opencv.hpp>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpSimulatorPioneer.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>


using namespace std;



int main(int argc, char **argv)
{
    vpHomogeneousMatrix cdMo;
    cdMo[0][3] = 0.0;  // x
    cdMo[1][3] = 0.0;  // y
    cdMo[2][3] = 1.;  // z

    vpHomogeneousMatrix cMo;
    cMo[0][3] = 0.3;         // x
    cMo[1][3] = cdMo[1][3];  // y
    cMo[2][3] = 10.;         // z
    vpRotationMatrix cRo(0, atan2(cMo[0][3], cMo[1][3]), 0);   // rot_y
    cMo.insert(cRo);

    vpFeaturePoint3D feature_x_z_;
    vpFeaturePoint3D feature_x_z_d_;
    feature_x_z_.buildFrom(cMo[0][3], cMo[1][3], cMo[2][3]);
    feature_x_z_d_.buildFrom(cdMo[0][3], cdMo[1][3], cdMo[2][3]);

    vpSimulatorPioneer robot;
    robot.setSamplingTime(0.04);
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
    task.setLambda(0.2);
    task.addFeature(feature_x_z_, feature_x_z_d_);  //, vpFeaturePoint3D::selectX() | vpFeaturePoint3D::selectZ()

    vpVelocityTwistMatrix cVe;
    cVe = robot.get_cVe();
    task.set_cVe(cVe);

    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);


    vpPlot graph(2, 800, 500, 400, 10, "Curves...");
    graph.initGraph(0, 2); // v. w
    graph.initGraph(1, 3); // error
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");

    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "wz");
    graph.setLegend(1, 0, "x");
    graph.setLegend(1, 1, "y");
    graph.setLegend(1, 2, "z");

    vpColVector v;
    int n = 0;
    // cMo
    while(true)
    {
        robot.getPosition(wMc);
        cMo = wMc.inverse() * wMo;

        feature_x_z_.buildFrom(cMo[0][3], cMo[1][3], cMo[2][3]);

        v = task.computeControlLaw();

        robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);
        graph.plot(0, n, v);
        graph.plot(1, n, task.getError()); // plot error vector


        double error = task.getError().sumSquare();
        std::cout << "n: " << n++ << " v: " << v[0] <<" " << v[1] << " error: " << error << std::endl;

        if (error < 0.0001) {
            std::cout << "Reached a small error. We stop the loop... " << std::endl;
            break;
        }
    }
    getchar();
    return 0;
}
