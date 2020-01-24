package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 *
 * 2018/09/30 Copied from OpenCvExampleBlueVisionDemo.java to experiment with various parameters.
 */
//@Disabled
@Autonomous(name = "Auto SkyStone Blue", group = "Test")
public class AutoSkyStoneBlue extends MasterAutonomous
{
    SkystoneDetectionOpenCV OpenCV_detector;

    public double m1, m2, m3;
    Stone skystonePlacement = Stone.MIDDLE;
    enum Stone {
        RIGHT,MIDDLE,LEFT
    }

    boolean isDetectingSkystone = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initAuto();
        telemetry.clear();
        telemetry.update();

        waitForStart();
        telemetry.clear();

        autoReverseDrive = true;


        // Initialize the skystoneDetector
        OpenCV_detector = new SkystoneDetectionOpenCV();

        // fullscreen display:
        //   app crashes if screen orientation switches from portrait to landscape
        //   screen goes to sleep, and webcam turns off a few minutes after init, and after play
        OpenCV_detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        OpenCV_detector.enable();
        // loop until user presses 'PLAY'
        // show detected mineral position telemetry
        while (!isStarted())
        {
            telemetry.update();
        }

        // run after user presses 'PLAY'
        while (opModeIsActive())
        {
           imuMoveAuto(12,0,1,.2,3);
            imuMoveAuto(0,9,1,.2,3);
           sleep(400);


            telemetry.addData("Left stone: ", m1);
            telemetry.addData("Middle stone: ", m2);
            telemetry.addData("Right stone: ", m3);

            /*
             *  If the yellow filter value of the right stone is lower than that of the middle or
             *  lef t stones, then it has more black and is therefore the SkyStone.
             *
             *  Otherwise, if the yellow value of the middle stone is less than that of the left
             *  stone, the middle stone is the SkyStone.
             *
             *  Failing the first two options, the left stone is the SkyStone.
             */
            while(isDetectingSkystone){
                m1 = OpenCV_detector.getMean1();
                m2 = OpenCV_detector.getMean2();
                m3 = OpenCV_detector.getMean3();

                if(m1 < m2 && m1 < m3)
                {
                    telemetry.addData("Right stone is SkyStone", "");
                    skystonePlacement = Stone.RIGHT;
                    isDetectingSkystone = false;

                }
                else if(m2 < m3 && m2 < m1)
                {
                    telemetry.addData("Middle stone is SkyStone", "");
                    skystonePlacement = Stone.MIDDLE;
                    isDetectingSkystone = false;
                }
                else if(m3 < m1 && m3 < m2)
                {
                    telemetry.addData("Left stone is SkyStone", "");
                    skystonePlacement = Stone.LEFT;
                    isDetectingSkystone = false;
                }
            }


            telemetry.update();

            collectSkystone(skystonePlacement);
            break;


        }

    }

    private void collectSkystone(Stone direction) throws InterruptedException {

        double referenceAngle = imu.getAngularOrientation().firstAngle;

        switch (direction){
            case LEFT:
                //imuPivot(imu.getAngularOrientation().firstAngle,90,.4,1,1);
                runIntake();
                imuMoveAuto(-5,0,1,.4,3);
                break;
            case MIDDLE:
                //imuPivot(imu.getAngularOrientation().firstAngle,90,.4,1,1);
                runIntake();
                imuMoveAuto(0,0,1,.4,3);
                break;
            case RIGHT:

                imuMoveAuto(0,-12,1,.3,3);
                imuPivot(referenceAngle,90,.35,1,4);
                imuMoveAuto(0,-15,1,.3,3);
                moveBackAndIntake();
                imuMoveAuto(0,13,1,.3,3);
                imuPivot(referenceAngle,0,.35,1,4);

                imuMoveAuto(0,89,1,.4,4);
                imuPivot(referenceAngle,-90,.35,1,4);

                imuMoveAuto(0,13,1,.3,3);
                grabbersDown();
                sleep(150);
                imuPivot(referenceAngle,-60,.5,1,1);
                imuMoveAuto(0 ,-23 ,1,.5,3);
                imuPivot(referenceAngle,0,.5,1,1);
                imuMoveAuto(0 ,14 ,1,.5,3);
                grabbersUp();

                moveLift(-1100);
                sleep(500);
                clawOut();
                sleep(600);

                clawUp();
                sleep(200);
                clawIn();
                sleep(200);
                moveLift(1500);

                imuMoveAuto(8 ,0 ,1,.2,3);
                imuMoveAuto(0,-44,1,.2,3);




                break;


        }

    }



}
