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
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorPioneer.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

int main()
{
  try
  {
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.04);
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    vpHomogeneousMatrix wMc, wMcd;
    wMcd[0][3] = 1;
    wMcd[1][3] = 1;
    wMcd[2][3] = 1;
    wMc.buildFrom(1,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(5));

//    wMc[0][3] = 0;
//    wMc[1][3] = 0;
//    wMc[2][3] = 1;

    robot.setPosition(wMc);

    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
    task.setLambda(0.2);

    vpHomogeneousMatrix cMcd = wMc.inverse() * wMcd;    // eMo

    vpFeaturePoint3D feature_x_y, feature_x_y_d;
    feature_x_y.buildFrom(cMcd[0][3], cMcd[1][3], 1);
    feature_x_y_d.buildFrom(0, 0, 1);
    task.addFeature(feature_x_y, feature_x_y_d,
                    vpFeaturePoint3D::selectX() | vpFeaturePoint3D::selectY());



    vpFeaturePointPolar feature_theta, feature_theta_d;
    vpColVector px = cMcd.getRotationMatrix().getCol(0);
    double theta = atan2(px[1], px[0]);
    double theta_desired = 0;
    feature_theta.set_rhoThetaZ(1, theta, 1);
    feature_theta_d.set_rhoThetaZ(1, theta_desired, 1);
    task.addFeature(feature_theta, feature_theta_d,
                                    vpFeaturePointPolar::selectTheta());



#ifdef VISP_HAVE_DISPLAY
    // Create a window (800 by 500) at position (400, 10) with 3 graphics
    vpPlot graph(4, 800, 900, 400, 10, "Curves...");

    // Init the curve plotter
    graph.initGraph(0, 3);
    graph.initGraph(1, 3);
    graph.initGraph(2, 1);
    graph.initGraph(3, 3);
    graph.setTitle(0, "Velocities");
    graph.setTitle(1, "Error s-s*");
    graph.setTitle(2, "Depth");
    graph.setTitle(3, "RobotTrajectory");
    graph.setLegend(0, 0, "vx");
    graph.setLegend(0, 1, "vy");
    graph.setLegend(0, 2, "wz");
    graph.setLegend(1, 0, "x-x*");
    graph.setLegend(1, 1, "y-y*");
    graph.setLegend(2, 0, "Z");
    graph.setLegend(3, 0, "camera");
    graph.setLegend(3, 1, "object");
    graph.setLegend(3, 2, "object2");

    // graph.setThickness(3, 2, 10);

    //    std::cout<<"wMo: ("<<wMo[0][3]<<","<<wMo[2][3]<<")"<<std::endl;
    double cross_size = 1e-1;
    graph.plot(3, 1, wMcd[0][3] - cross_size, wMcd[1][3] - cross_size);
    graph.plot(3, 1, wMcd[0][3] + cross_size, wMcd[1][3] + cross_size);
    graph.plot(3, 1, wMcd[0][3], wMcd[1][3]);
    graph.plot(3, 1, wMcd[0][3] - cross_size, wMcd[1][3] + cross_size);
    graph.plot(3, 1, wMcd[0][3] + cross_size, wMcd[1][3] - cross_size);

//    graph.plot(3, 2, wMo2[0][3] - cross_size, wMo2[1][3] - cross_size);
//    graph.plot(3, 2, wMo2[0][3] + cross_size, wMo2[1][3] + cross_size);
//    graph.plot(3, 2, wMo2[0][3], wMo2[1][3]);
//    graph.plot(3, 2, wMo2[0][3] - cross_size, wMo2[1][3] + cross_size);
//    graph.plot(3, 2, wMo2[0][3] + cross_size, wMo2[1][3] - cross_size);
#endif

    int iter = 0;
    int max_iter = 5000;
    for (; iter < max_iter;)
    {
//      std::cout << "111" << std::endl;

      cMcd = wMc.inverse() * wMcd;
      feature_x_y.buildFrom(cMcd[0][3], cMcd[1][3], 1);
      feature_x_y_d.buildFrom(0, 0, 1);

      px = cMcd.getRotationMatrix().getCol(0);
      theta = atan2(px[1], px[0]);
      feature_theta.set_rhoThetaZ(1, theta, 1);
      feature_theta_d.set_rhoThetaZ(1, theta_desired, 1);

      vpVelocityTwistMatrix cVe;
      cVe.eye();
      task.set_cVe(cVe);
      vpMatrix eJe(6, 3);
      eJe = 0;
      eJe[0][0] = 1;
      eJe[1][1] = 1;
      eJe[5][2] = 1;
      task.set_eJe(eJe);

      vpColVector v_calc = task.computeControlLaw();
      // vpColVector v1(2);
      // v1[0] = 0;
      // v1[1] = v[0];
      vpColVector v(6);
      v[0] = v_calc[0];
      v[1] = v_calc[1];
      v[5] = v_calc[2];
      std::cout << "v_calc:" << v_calc.transpose() << std::endl;
      std::cout << "v:" << v.transpose() << std::endl;
      // vpColVector v = task.computeControlLaw(iter*robot.getSamplingTime());
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);
      wMc = robot.getPosition();

#ifdef VISP_HAVE_DISPLAY
      graph.plot(0, iter, v_calc); // plot velocities applied to the robot
      graph.plot(1, iter, task.getError()); // plot error vector
      // graph.plot(2, 0, iter, Z); // plot the depth

      // std::cout << "error: " << task.getError().transpose() << std::endl;

      //      std::cout<<"wMc:"<<std::endl;
      //      wMc.print();
      //      std::cout<<std::endl;

      //      vpColVector error = task.getError();
      //      std::cout<<"error:\n"<<error<<std::endl;

      //      vpHomogeneousMatrix cMe = robot.get_cMe();
      //      vpHomogeneousMatrix wMe = wMc*cMe;

      vpColVector px(4), pz(4), px_w(4);
      px[0] = 0.1;
      px[1] = 0;
      px[2] = 0;
      px[3] = 1;

      //            pz[0] = 0;
      //            pz[1] = 0;
      //            pz[2] = 0.3;
      //            pz[3] = 1;

      // px_w = wMc*pz;
      px_w = wMc * px;

      //      graph.plot(3, 0, wMc[0][3], wMc[2][3]);
      //      graph.plot(3, 0, px_w[0]/px_w[3], px_w[2]/px_w[3]);
      //      graph.plot(3, 0, wMc[0][3], wMc[2][3]);

      graph.plot(3, 0, wMc[0][3], wMc[1][3]);
      graph.plot(3, 0, px_w[0], px_w[1]);
      graph.plot(3, 0, wMc[0][3], wMc[1][3]);

#endif

      iter++;

      if (task.getError().sumSquare() < 0.0001)
      {
        std::cout << "Reached a small error. We stop the loop... " << std::endl;
        break;
      }

      vpTime::sleepMs(10);
    }
    task.print();

#ifdef VISP_HAVE_DISPLAY
    graph.saveData(0, "./v2.dat");
    graph.saveData(1, "./error2.dat");
    graph.saveData(2, "./depth1.dat");

    const char* legend = "Click to quit...";
    vpDisplay::displayText(graph.I, (int)graph.I.getHeight() - 60,
                           (int)graph.I.getWidth() - 150, legend, vpColor::red);
    vpDisplay::flush(graph.I);
    vpDisplay::getClick(graph.I);
#endif

    // Kill the servo task
    task.kill();
  }
  catch (vpException& e)
  {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
