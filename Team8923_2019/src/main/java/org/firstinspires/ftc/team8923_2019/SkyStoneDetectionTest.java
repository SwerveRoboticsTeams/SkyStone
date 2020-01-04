package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name = "Skystone Detection Test", group = "Test")
public class SkyStoneDetectionTest extends LinearOpMode
{

    //Dogeforia6220 vuforia;
//    WebcamName webcamName;
    // DogeCV skystoneDetector
    SkystoneDetectionOpenCV OpenCV_detector;

    public double m1, m2, m3;

    @Override
    public void runOpMode() throws InterruptedException
    {
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

        waitForStart();

        // run after user presses 'PLAY'
        while (opModeIsActive())
        {
            m1 = OpenCV_detector.getMean1();
            m2 = OpenCV_detector.getMean2();
            m3 = OpenCV_detector.getMean3();

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
            if(m1 < m2 && m1 < m3)
            {
                telemetry.addData("Left stone is SkyStone", "");
            }
            else if(m2 < m3)
            {
                telemetry.addData("Middle stone is SkyStone", "");
            }
            else
            {
                telemetry.addData("Right stone is SkyStone", "");
            }

            telemetry.update();
        }

    }

}
