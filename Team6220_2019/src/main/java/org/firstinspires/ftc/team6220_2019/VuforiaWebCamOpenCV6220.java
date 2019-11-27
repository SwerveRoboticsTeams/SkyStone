package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

public class VuforiaWebCamOpenCV6220 extends LinearOpMode{

    Dogeforia6220 vuforia;
    WebcamName webcamName;
    // DogeCV OpenCV_detector
    SkystoneDetectionOpenCV OpenCV_detector;

    public double m1, m2, m3;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Default webcam name
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Set up parameters for Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key - Truong
        parameters.vuforiaLicenseKey = "AdTQ1zz/////AAABmcsU0JSsfUAAnZVqALQznloFPRzK4IDs9AKHiU80F9ncKlBHZBPTN3XWUSLbcsKUfy/iW4P/y64OCRHrAiUTE430LhFnx8rGRtKUv8P03XTaE11Xj9gbN5vThAIBcrnk/CovUIBFJjptCseciz/akh2mWHAlNznx5kWdP0QbFRi9i6fZffoHXaBXYERvzyK/wYYxMLuwVL+qBGIuRzJRS4f2b6RZ8cq/SEs6Ulfg5HQgV24KqFA65+T7iGXKCrdQMi0eUN0Oc4DmrKrHKF55bEtA108/jh8cz1tAwsrAvjle6JUX+yUQ4RDX8Zv/GpuWdek3VFGxumvh8EdQAmZqUmaWdcrpHLXMcftKdOjyvyUf";
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia6220 object
        vuforia = new Dogeforia6220(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Initialize the OpenCV_detector
        OpenCV_detector = new SkystoneDetectionOpenCV();

        // fullscreen display:
        //   app crashes if screen orientation switches from portrait to landscape
        //   screen goes to sleep, and webcam turns off a few minutes after init, and after play
        OpenCV_detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);

        // Set the OpenCV_detector and enable it.
        vuforia.setDogeCVDetector(OpenCV_detector);
        vuforia.enableDogeCV();
        vuforia.start();

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

            telemetry.addData("Right stone: ", m1);
            telemetry.addData("Middle stone: ", m2);
            telemetry.addData("Left stone: ", m3);

            /*
             *  If the yellow filter value of the right stone is lower than that of the middle or
             *  left stones, then it has more black and is therefore the SkyStone.
             *
             *  Otherwise, if the yellow value of the middle stone is less than that of the left
             *  stone, the middle stone is the SkyStone.
             *
             *  Failing the first two options, the left stone is the SkyStone.
             */
            if(m1 < m2 && m1 < m3)
            {
                telemetry.addData("Right stone is SkyStone", "");
            }
            else if(m2 < m3)
            {
                telemetry.addData("Middle stone is SkyStone", "");
            }
            else
            {
                telemetry.addData("Left stone is SkyStone", "");
            }

            telemetry.update();
        }

        // stop the vision system
        vuforia.stop();
    }

}
