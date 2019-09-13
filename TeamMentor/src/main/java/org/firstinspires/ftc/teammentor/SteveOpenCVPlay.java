/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teammentor;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * This OpMode illustrates the basics of using opencv with an image obtained using Vuforia.
 *
 */

@TeleOp(name="steve opencv", group ="opencv")
//@Disabled
public class SteveOpenCVPlay extends LinearOpMode {

    public static final String TAG = "opencv with vuforia Sample";

    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        initializeVuforia(true);

        telemetry.addData(">", "Wait for Vuforia to start then Press Play");
        telemetry.update();

        CameraViewDisplay view = CameraViewDisplay.getInstance();

        waitForStart();


        while (opModeIsActive()) {


            Bitmap bitmap = getBitmapFromVuforia();
            Mat mat  = getMatFromBitmap(bitmap);

            if (mat != null)
            {
                //do opencv operations here
                //telemetry.addData("image", "image found");

                //the vuforia image is 1280 wide by 720 tall
                //let's divide that image into 3rds, so our regions are roughly
                //x: 0..426, 427..854, 855..1279
                //y: should we start with full height?

                int numCirclesRegion1 = findCircles(mat, 0, 0, 426, 719, true);
                int numCirclesRegion2 = findCircles(mat, 427, 0, 854, 719, true);
                int numCirclesRegion3 = findCircles(mat, 855, 0, 1279, 719, true);

                telemetry.addData("region 1: ", numCirclesRegion1);
                telemetry.addData("region 2: ", numCirclesRegion2);
                telemetry.addData("region 3: ", numCirclesRegion3);

            }
            else
            {
                telemetry.addData("image", "image NOT found");
            }


            
            telemetry.update();
        }
    }


    //This function will try to detect the presence of circles in an image
    int findCircles(Mat source, int x1, int y1, int x2, int y2, boolean displayResult)
    {
        //Create a temporary Mat to hold a copy of a sub-region of the source image
        //You'll generally want to work with copies so you're not modifying the source image
        Mat region = new Mat();

        //use a method to get a sub-region of the source
        //note that openCV functions use the order "row, col", which is backwards from the typical "x, y" order
        region = source.submat(y1, y2, x1, x2);

        //you can display the region if you want to check it
        //display(region);

        //Edge detection works better if you:
        // 1) use a grayscale image and
        // 2) reduce the noise in the image by blurring it
        //So, we'll create another copy of the image for those steps
        Mat grayblur = new Mat();
        //make a grayscale copy
        Imgproc.cvtColor(region, grayblur, Imgproc.COLOR_BGR2GRAY);
        //display(grayblur);

        //Blur the image in-place (overwriting the gray image)
        //You can tweak the Size parameter to make the image more or less blurry
        Imgproc.GaussianBlur(grayblur, grayblur, new Size(5,5), 0);

        //There are other techniques that can be applied to the image to improve detection,
        //such as creating a threshold. These steps are left for a future exercise
        //because I haven't figured out how to make them work yet ;) and the detection
        //is working well enough without them so far
        //Imgproc.threshold(grayblur, grayblur, 150, 255, Imgproc.THRESH_BINARY);

        //display(grayblur);

        //Create yet another Mat into which we will store the locations of any circles that we find.
        //Remember, Mats are really matrices, so they can hold other things besides just image data.
        Mat circles = new Mat();

        //Use an openCV algorithm for trying to detect circles.
        //The key params to adjust are the last two numbers.
        Imgproc.HoughCircles(grayblur, circles, Imgproc.CV_HOUGH_GRADIENT, 1.8, 10);

        //iterate through the circles that we found (if any) and draw them on the region image
        for (int i = 0; i < circles.cols(); i++) {
            double[] vCircle = circles.get(0, i);

            Point pt = new Point(Math.round(vCircle[0]), Math.round(vCircle[1]));
            int radius = (int)Math.round(vCircle[2]);

            //draw a circle on an image. The Scalar is the color.
            //Note that the color order in openCV is always B,G,R, so (0,0,255) is Red.
            Imgproc.circle(region, pt, radius, new Scalar(0, 0, 255), 2);
        }

        //if (displayResult) display(region);

        //return the number of circles that we found
        return (circles.cols());

    }

    //use an openCV function to display an image on the screen
    int display(Mat img) {
        return display(2000, img);
    }

    //use an openCV function to display an image on the screen for a given amount of time
    int display(int delayInMS, Mat img) {
        //HighGui.imshow( "demo", img );
        //int c = HighGui.waitKey( delayInMS );
        //if (c >= 0) { return -1; }
        return 0;
    }




    public Bitmap getBitmapFromVuforia()
    {
        VuforiaLocalizer.CloseableFrame frame = null; //used for storing info from vuforia frame queue to be analyzed in other methods
        Image image = null;  //an image that we will obtain from vuforia
        Bitmap bitMap = null; //a bitmap representation of that image
        int imageFormat = 0; //the format of the received image
        long numImages = 0; //the number of images obtained
        boolean goodImageFound = false;

        try { frame = vuforia.getFrameQueue().take(); }
        catch (Exception e) {return null;}

        if (frame==null) return null;

        numImages = frame.getNumImages();
        if (numImages==0) return null;

        for (int j = 0; j < numImages; j++)
        {
            image = frame.getImage(j);
            imageFormat = image.getFormat();

            if (imageFormat == PIXEL_FORMAT.RGB565) {
                goodImageFound = true;
                break;
            }
        }

        if (!goodImageFound) return null;

        int imageWidth = image.getWidth();
        int imageHeight = image.getHeight();

        // Create bitmap of image to detect color
        bitMap = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
        bitMap.copyPixelsFromBuffer(image.getPixels());

        //close the frame, prevents memory leaks and crashing
        frame.close();

        return bitMap;
    }

    public Mat getMatFromBitmap(Bitmap b)
    {
        if (b==null) return null;

        //put the image into a MAT for OpenCV
        Mat tmp = new Mat(); //b.getWidth(), b.getHeight, CvType.CV_8UC4
        Utils.bitmapToMat(b, tmp);

        return tmp;
    }

    public void initializeVuforia(boolean showPreview)
    {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters;

        if (showPreview) parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        else parameters = new VuforiaLocalizer.Parameters(); //no preview

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //This licence key belongs to Steve Geffner
        parameters.vuforiaLicenseKey = "ATJf0AL/////AAAAGQZ9xp9L+k5UkmHj3LjxcoQwNTTBJqjO9LYsbkWQArRpYKQmt7vqe680RCQSS9HatStn1XZVi7rgA8T7qrJz/KYI748M4ZjlKv4Z11gryemJCRA9+WWkQ51D3TuYJbQC46+LDeMfbvcJQoQ79jtXr7xdFhfJl1mRxf+wMVoPWfN6Dhr8q3XVxFwOE/pM3gXWQ0kacbcGR/vy3NAsbOhf02DEe5WoV5PNZTF34LWN3dWURu7NJsnbFzkpzXdogeVAdiQ3QUWDvuhEwvSJY4W+fCTb15t6T/c/GJ/vqptsVKqavXk6MQobnUsVFpFP+5OSuRQe7EgvWuOxn7xn5YlC+CWAYh9LrXDpktwCwBAiX3Gx";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        // Setup for getting pixel information
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        // Make sure that vuforia doesn't begin racking up unnecessary frames
        vuforia.setFrameQueueCapacity(1);
    }


    /* A function and supporting variable to debounce the X button on the gamepad,
    *  so that our code only runs one time for each button press */
    boolean xButtonAlreadyDown = false;  //need a variable to debounce the button press
    public boolean gamepad1XButtonPressed()
    {
        if (gamepad1.x & !xButtonAlreadyDown)
        {
            xButtonAlreadyDown = true; //debounce the button so this code only runs one time per button press
            return true;
        }
        else if (!gamepad1.x & xButtonAlreadyDown) xButtonAlreadyDown = false; //clear our debounce flag

        return false;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
