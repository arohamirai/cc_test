/*!
  \example tutorial-simu-pioneer.cpp

  Example that shows how to simulate a visual servoing on a Pioneer mobile robot
  equipped with a camera.
  The current visual features that are used are s = (x, log(Z/Z*)). The desired
  one are s* = (x*, 0), with:
  - x the abscisse of the point measured at each iteration
  - x* the desired abscisse position of the point (x* = 0)
  - Z the depth of the point measured at each iteration
  - Z* the desired depth of the point equal to the initial one.

  The degrees of freedom that are controlled are (vx, wz), where wz is the
  rotational velocity
  and vx the translational velocity of the mobile platform at point M located at
  the middle
  between the two wheels.

  The feature x allows to control wy, while log(Z/Z*) allows to control vz.

  */
#include <iostream>

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeaturePointPolar.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpSimulatorPioneer.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>
#include <unistd.h>

using namespace std;

int main()
{

    vpHomogeneousMatrix camera_init_pose_in_world, camera_final_pose_in_world, wMc;
    vpHomogeneousMatrix cMo, cdMo;

    cdMo[0][3] = 0;
    cdMo[1][3] = 0;
    cdMo[2][3] = 1;

    vpTranslationVector t;
    vpRzyxVector r;
    t.buildFrom(0, 0, 1);
    r.buildFrom(vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    camera_init_pose_in_world.buildFrom(t, vpRotationMatrix(r));
    t.buildFrom(1, 1, 1);
    r.buildFrom(vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    camera_final_pose_in_world.buildFrom(t, vpRotationMatrix(r));

    cMo = camera_init_pose_in_world.inverse() * camera_final_pose_in_world;

    vpFeaturePoint3D p, pd;
    p.buildFrom(cMo[0][3], cMo[1][3], 1);
    pd.buildFrom(cdMo[0][3], cdMo[1][3], 1);

    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
    task.setLambda(0.5);
    task.addFeature(p, pd, vpFeaturePoint3D::selectX() | vpFeaturePoint3D::selectY());

    vpFeaturePointPolar feature_theta, feature_theta_d;
    double theta, theta_desired;
    theta = std::atan2(cMo[1][0], cMo[0][0]);
    theta_desired = 0;
    feature_theta.set_rhoThetaZ(1, theta, 1);
    feature_theta_d.set_rhoThetaZ(1, theta_desired, 1);
    task.addFeature(feature_theta,feature_theta_d, vpFeaturePointPolar::selectTheta());


    vpPlot graph(3, 800, 500, 400, 10, "Curves...");
    graph.initGraph(0, 6);  //v
    graph.initGraph(1, 1);  //error
    graph.initGraph(2, 2);  //traj
    graph.setTitle(0, "vel");
    graph.setTitle(1, "s - s*");
    graph.setTitle(2, "traj");
    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "vy");
    graph.setLegend(0, 2, "vz");
    graph.setLegend(0, 3, "wx");
    graph.setLegend(0, 4, "wy");
    graph.setLegend(0, 5, "wz");
    graph.setLegend(1, 0, "error");
    graph.setLegend(2, 0, "x-y");


    vpSimulatorCamera robot;
    robot.setSamplingTime(0.040);
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    robot.setPosition(camera_init_pose_in_world);
    for(int iter = 0; iter < 500; iter++)
    {
        robot.getPosition(wMc);
        cMo = wMc.inverse() * camera_final_pose_in_world;

        p.buildFrom(cMo[0][3], cMo[1][3], 1);
        theta = std::atan2(cMo[1][0], cMo[0][0]);
        feature_theta.set_rhoThetaZ(1, theta, 1);


        vpVelocityTwistMatrix cVe;
        cVe.eye();
        task.set_cVe(cVe);
        vpMatrix eJe(6, 3);
        eJe = 0;
        eJe[0][0] = 1;
        eJe[1][1] = 1;
        eJe[5][2] = 1;
        task.set_eJe(eJe);

        vpColVector v_cal = task.computeControlLaw();

        vpColVector v(6);
        v[0] = v_cal[0];
        v[1] = v_cal[1];
        //v[5] = v_cal[2];
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);

        vpColVector error = task.getError();
        vpColVector error_display(1);
        error_display[0] = error.sumSquare();

        graph.plot(0, iter, v);
        graph.plot(1, iter, error_display);
        graph.plot(2, 0, wMc[0][3], wMc[1][3]);


        vpColVector px(4), pz(4), px_w(4);
        px[0] = 0.1;
        px[1] = 0;
        px[2] = 0;
        px[3] = 1;
        px_w = wMc * px;

        graph.plot(2, 1, wMc[0][3], wMc[1][3]);
        graph.plot(2, 1, px_w[0], px_w[1]);
        graph.plot(2, 1, wMc[0][3], wMc[1][3]);

        if( error_display[0]< 0.0001)
        {
            std::cout << "reach an error, we will stop" << std::endl;
            break;
        }

        usleep(0.1*10e5);

    }

    getchar();

//
//
//
//
//        vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
//        vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));
//        vpPoint point[4];
//        point[0].setWorldCoordinates(-0.1, -0.1, 0);
//        point[1].setWorldCoordinates(0.1, -0.1, 0);
//        point[2].setWorldCoordinates(0.1, 0.1, 0);
//        point[3].setWorldCoordinates(-0.1, 0.1, 0);
//        vpServo task;
//        task.setServo(vpServo::EYEINHAND_CAMERA);
//        task.setInteractionMatrixType(vpServo::CURRENT);
//        task.setLambda(0.5);
//        vpFeaturePoint p[4], pd[4];
//        for (unsigned int i = 0; i < 4; i++) {
//            point[i].track(cdMo);
//            vpFeatureBuilder::create(pd[i], point[i]);
//            point[i].track(cMo);
//            vpFeatureBuilder::create(p[i], point[i]);
//            task.addFeature(p[i], pd[i]);
//        }
//        vpHomogeneousMatrix wMc, wMo;
//        vpSimulatorCamera robot;
//        robot.setSamplingTime(0.040);
//        robot.getPosition(wMc);
//
//        wMo = wMc * cMo;
//
//        //return 0;
//
//
//        vpPlot graph(1, 800, 500, 400, 10, "Curves...");
//
//        graph.initGraph(0, 6); // v. w
//        //graph.initRange(0, 0, 15, 0, 1);
//        graph.setTitle(0, "traj");
//        graph.setLegend(0, 0, "vx");
//        graph.setLegend(0, 1, "vy");
//        graph.setLegend(0, 2, "vz");
//        graph.setLegend(0, 3, "wx");
//        graph.setLegend(0, 4, "wy");
//        graph.setLegend(0, 5, "wz");
//
//
//
//        for (unsigned int iter = 0; iter < 150; iter++) {
//            robot.getPosition(wMc);
//            cMo = wMc.inverse() * wMo;
//            for (unsigned int i = 0; i < 4; i++) {
//                point[i].track(cMo);
//                vpFeatureBuilder::create(p[i], point[i]);
//            }
//            vpColVector v = task.computeControlLaw();
//            robot.setVelocity(vpRobot::CAMERA_FRAME, v);
//
//            graph.plot(0, iter, v);
//            //std::cout << v.size();
//
//
//        }
//        task.kill();
//    getchar();
}
