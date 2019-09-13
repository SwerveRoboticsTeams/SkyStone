package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

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
@TeleOp(name="Vuforia WebCam OpenCV Demo")
//@Disabled
public class VuforiaWebCamOpenCV extends LinearOpMode {

    Dogeforia vuforia;
    WebcamName webcamName;
    // DogeCV OpenCV_detector
    TruongOpenCVExample OpenCV_detector;

    @Override
    public void runOpMode()
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

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Initialize the OpenCV_detector
        OpenCV_detector = new TruongOpenCVExample();

        // fullscreen display:
        //   app crashes if screen orientation switches from portrait to landscape
        //   screen goes to sleep, and webcam turns off a few minutes after init, and after play
        //OpenCV_detector.init(hardwareMap.appContext, ActivityViewDisplay.getInstance(), 0, true);

        OpenCV_detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        OpenCV_detector.SetThreshold(80);
        OpenCV_detector.setShowCountours(false);
        // Set the OpenCV_detector
        vuforia.setDogeCVDetector(OpenCV_detector);
        vuforia.enableDogeCV();
        // don't show Vuforia vuforia.showDebug();
        vuforia.start();

        // loop until user presses 'PLAY'
        // show detected mineral position telemetry
        while (!isStarted())
        {
            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2), (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2) ) );
            telemetry.update();
        }

        waitForStart();

        // run after user presses 'PLAY'
        while (opModeIsActive())
        {
            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2), (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2) ) );
            telemetry.update();
        }

        // stop the vision system
        vuforia.stop();

    }

}
