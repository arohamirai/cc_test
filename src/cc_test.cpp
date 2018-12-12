#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>

int main()
{
        vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
        vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));
        vpPoint point[4];
        point[0].setWorldCoordinates(-0.1, -0.1, 0);
        point[1].setWorldCoordinates(0.1, -0.1, 0);
        point[2].setWorldCoordinates(0.1, 0.1, 0);
        point[3].setWorldCoordinates(-0.1, 0.1, 0);
        vpServo task;
        task.setServo(vpServo::EYEINHAND_CAMERA);
        task.setInteractionMatrixType(vpServo::CURRENT);
        task.setLambda(0.5);
        vpFeaturePoint p[4], pd[4];
        for (unsigned int i = 0; i < 4; i++) {
            point[i].track(cdMo);
            vpFeatureBuilder::create(pd[i], point[i]);
            point[i].track(cMo);
            vpFeatureBuilder::create(p[i], point[i]);
            task.addFeature(p[i], pd[i]);
        }
        vpHomogeneousMatrix wMc, wMo;
        vpSimulatorCamera robot;
        robot.setSamplingTime(0.040);
        robot.getPosition(wMc);

        wMo = wMc * cMo;

        //return 0;


        vpPlot graph(1, 800, 500, 400, 10, "Curves...");

        graph.initGraph(0, 6); // v. w
        //graph.initRange(0, 0, 15, 0, 1);
        graph.setTitle(0, "traj");
        graph.setLegend(0, 0, "vx");
        graph.setLegend(0, 1, "vy");
        graph.setLegend(0, 2, "vz");
        graph.setLegend(0, 3, "wx");
        graph.setLegend(0, 4, "wy");
        graph.setLegend(0, 5, "wz");



        for (unsigned int iter = 0; iter < 150; iter++) {
            robot.getPosition(wMc);
            cMo = wMc.inverse() * wMo;
            for (unsigned int i = 0; i < 4; i++) {
                point[i].track(cMo);
                vpFeatureBuilder::create(p[i], point[i]);
            }
            vpColVector v = task.computeControlLaw();
            robot.setVelocity(vpRobot::CAMERA_FRAME, v);

            graph.plot(0, iter, v);
            //std::cout << v.size();


        }
        task.kill();
    getchar();
}