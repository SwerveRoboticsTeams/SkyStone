package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

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
@Disabled
@Autonomous(name = "Auto SkyStone Red", group = "Test")
public class AutoSkyStoneRed extends MasterAutonomous
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

        autoReverseDrive = false;

        OpenCV_detector = new SkystoneDetectionOpenCV();
        OpenCV_detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        OpenCV_detector.enable();

        while (!isStarted())
        {
            telemetry.update();
        }

        waitForStart();

        // run after user presses 'PLAY'
        while (opModeIsActive())
        {
           imuMoveAuto(-12,7,1,.2,3);
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
                    telemetry.addData("Left stone is SkyStone", "");
                    skystonePlacement = Stone.LEFT;
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
                    telemetry.addData("Right stone is SkyStone", "");
                    skystonePlacement = Stone.RIGHT;
                    isDetectingSkystone = false;
                }
            }


            telemetry.update();

            collectSkystone(skystonePlacement);
            break;


        }

    }

    private void collectSkystone(Stone direction) throws InterruptedException {

        switch (direction){
            case LEFT:
                break;
            case MIDDLE:
                break;
            case RIGHT:
                imuMoveAuto(30.5,3.5,1,.4,3);
                moveBackAndIntake();
                imuPivot(imu.getAngularOrientation().firstAngle,30,.3,1,1);
                imuMoveAuto(0,30,1,.4,3);
                imuPivot(imu.getAngularOrientation().firstAngle,-30,.3,1,1);
                imuMoveAuto(0,63,1,.4,3);
                break;


        }

    }



}