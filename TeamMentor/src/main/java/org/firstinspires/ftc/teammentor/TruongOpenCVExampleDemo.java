package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
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
@TeleOp(name="Example: Truong OpenCV Demo")
//@Disabled
public class TruongOpenCVExampleDemo extends LinearOpMode {
    private TruongOpenCVExample TruongVision;
    @Override
    public void runOpMode() {
        TruongVision = new TruongOpenCVExample();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        TruongVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // fullscreen display
        //TruongVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        TruongVision.setShowCountours(false);

        // start the vision system
        TruongVision.enable();

        waitForStart();

        TruongVision.setShowCountours(true);

        while (opModeIsActive()) {
            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (TruongVision.getGoldRect().x + TruongVision.getGoldRect().width / 2), (TruongVision.getGoldRect().y + TruongVision.getGoldRect().height / 2) ) );
            telemetry.update();


        }

        // stop the vision system
        TruongVision.disable();

    }

}
