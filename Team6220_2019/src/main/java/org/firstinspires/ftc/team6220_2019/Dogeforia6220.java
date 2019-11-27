// This file came from DogeCV library from Wizards.exe
// package com.disnodeteam.dogecv;
// Removed Vuforia tracking because it's not used
// Need Vuforia to access the webcam

package org.firstinspires.ftc.team6220_2019;

import android.app.Activity;
import android.graphics.Bitmap;
import android.util.Log;

import com.vuforia.Frame;

import org.corningrobotics.enderbots.endercv.DrawViewSource;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//import com.disnodeteam.dogecv.detectors.DogeCVDetector;

/**
 * An implementation of Vuforia intended to be cross-compatible with OpenCV (and DogeCV by extension)
 */

public class Dogeforia6220 extends VuforiaLocalizerImpl {
    private SkystoneDetectionOpenCV OpenCV_detector;

    DrawViewSource displayView;
    boolean dogeCVEnabled;
    boolean showDebug = false;

    Thread workerThread;
    Bitmap outputImage;
    Bitmap bitmap;
    Bitmap rotatedBitmap;
    Mat inputMat;
    Mat outMat;
    BlockingQueue<CloseableFrame> frames;

    MasterOpMode master;

    // Todo Finalize which members should be public and which should be private.
    // Vuforia objects that will be initialized in initVuforia() method.  We want some of ------------------------
    // them to be accessible to classes using this resource class.
    private VuforiaTrackables targetsSkyStone;

    VectorF translation = new VectorF(0, 0, 0);                                                                          //**Use this to navigate
    Orientation rotation = new Orientation();                                                                                     //**Use this to navigate

    // For convenience, gather together all the trackable objects in one easily-iterable collection */
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    VuforiaTrackable stoneTarget;
    VuforiaTrackable blueRearBridge;
    VuforiaTrackable redRearBridge;
    VuforiaTrackable redFrontBridge;
    VuforiaTrackable blueFrontBridge;
    VuforiaTrackable red1;
    VuforiaTrackable red2;
    VuforiaTrackable front1;
    VuforiaTrackable front2;
    VuforiaTrackable blue1;
    VuforiaTrackable blue2;
    VuforiaTrackable rear1;
    VuforiaTrackable rear2;

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQdgAgj/////AAABmZGzg951/0AVjcK/+QiLWG1Z1PfbTwUouhED8hlwM6qrpAncj4xoMYYOUDxF+kreiazigY0q7OMa9XeMyxNlEQvyMFdefVUGSReIxJIXYhFaru/0IzldUlb90OUO3+J4mGvnzrqYMWG1guy00D8EbCTzzl5LAAml+XJQVLbMGrym2ievOij74wabsouyLb2HOab5nxk0FycYqTWGhKmS7/h4Ddd0UtckgnHDjNrMN4jqk0Q9HeTa8rvN3aQpSUToubAmfXe6Jgzdh2zNcxbaNIfVUe/6LXEe23BC5mYkLAFz0WcGZUPs+7oVRQb7ej7jTAJGA6Nvb9QKEa9MOdn0e8edlQfSBRASxfzBU2FIGH8a";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (5.75f) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                           // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // todo Adjust these numbers for actual webcam position on robot.
    // todo Also:  Not working?
    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT = 8.8f * mmPerInch;   // eg: Camera is 8.8 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 4.8f * mmPerInch;   // eg: Camera is 4.8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT = -6.5f * mmPerInch;     // eg: Camera is 6.5 Inches right of the robot's center line

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    //------------------------------------------------------------------------------------------------------------



    public Dogeforia6220(Parameters parameters) {
        super(parameters);
    }

    public void setDogeCVDetector(SkystoneDetectionOpenCV OpenCV_detector){
        this.OpenCV_detector = OpenCV_detector;
        OpenCV_detector.enable();
        displayView = OpenCV_detector.getRawView();
        setMonitorViewParent(displayView.getId());
        setFrameQueueCapacity(1);
    }

    public void start(){
        workerThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(!workerThread.isInterrupted()){
                    render();
                }
            }
        });
        workerThread.setName("Dogeforia6220 Thread");
        workerThread.start();

        Log.d("DogeCV", workerThread.getState().toString());

    }

    public void enableDogeCV(){

        dogeCVEnabled = true;
    }

    public void disableDogeCV(){
        dogeCVEnabled = false;
    }
    public void enableTrack(){
        startTracker();
    }

    public void disableTrack() {
        stopTracker();
    }
    public void showDebug(){
        showDebug = true;

    }

    public void processFrame(Frame frame){
        if(frame != null ){

            bitmap = convertFrameToBitmap(frame);

            inputMat = new Mat(bitmap.getWidth(), bitmap.getHeight(), CvType.CV_8UC1);
            Utils.bitmapToMat(bitmap,inputMat);

            outMat = OpenCV_detector.processFrame(inputMat, null);

            if(!outMat.empty() ){
                // should not need this since outMat is same dim as inputMat
                //bitmap.setHeight(outMat.height());
                //bitmap.setWidth(outMat.width());

                Mat rotatedMat;
                rotatedMat = new Mat(outMat.height(), outMat.width(), CvType.CV_8UC1);
                Core.rotate(outMat, rotatedMat, Core.ROTATE_90_CLOCKWISE);

                // create bitmap, swap height, width for rotation
                rotatedBitmap = Bitmap.createBitmap(rotatedMat.width(), rotatedMat.height(), Bitmap.Config.RGB_565);
                rotatedBitmap.setHeight(rotatedMat.height());
                rotatedBitmap.setWidth(rotatedMat.width());
                Utils.matToBitmap(rotatedMat, rotatedBitmap);
                //Utils.matToBitmap(rotatedMat, bitmap);

                // adjusted height = <user-chosen width> * original height / original width
                int dispWidth = displayView.getWidth();      //656
                int dispHeight = displayView.getHeight();    // 1054
                int rotatedMatHeight = rotatedMat.height();  //800
                int rotatedMatWidth = rotatedMat.width();    // 448
                double adjustedWidth = rotatedMatWidth * dispHeight / dispWidth;  // 367
//                outputImage =  Bitmap.createScaledBitmap(rotatedBitmap, (int)adjustedWidth, dispWidth, false);
                outputImage =  Bitmap.createScaledBitmap(rotatedBitmap, (int)adjustedWidth, dispHeight, false);

                ((Activity)displayView.getContext()).runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        displayView.onFrame(outputImage);
//                        displayView.onFrame(rotatedBitmap);
                        displayView.invalidate();
                    }
                });

            }else{
                Log.w("DogeCV", "MAT BITMAP MISMATCH OR EMPTY ERROR");
            }


            inputMat.release();
            outMat.release();


        }else{
            Log.d("DogeCV", "No Frame!");
        }
    }

    public void render() {
       // Log.d("DogeCV", "Rendering Frame");
       // super.onRenderFrame()

        if(OpenCV_detector != null && dogeCVEnabled){

            if(!getFrameQueue().isEmpty()){
                try {
                    processFrame(getFrameQueue().take());
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }else{
                // the following line floods the log with messages; remove it
                //Log.w("DogeCV", "Frame is empty. Enabling AparnaCV: " + getFrameQueueCapacity());
            }

            /*
            getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
            {
                @Override public void accept(Frame frame)
                {
                    processFrame(frame);
                }
            }));
             */
        }

    }

    public void stop(){
        close();
        ((Activity)displayView.getContext()).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                workerThread.interrupt();

                OpenCV_detector.disable();
            }
        });

    }

    // Use this method to start Vuforia at the beginning of a match.
    public void initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = master.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", master.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = master.webcamName;
        parameters.useExtendedTracking = false;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        // Initialize image targets----------------------------------------------------------------------------------------------
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // Add targets to master list.
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Todo Does this need to be part of a separate List to prevent confusion between relative vs. absolute coordinates?
        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward (toward the audience), and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        //-----------------------------------------------------------------------------------------------------------------------


        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK)
        {
            phoneYRotate = -90;
        } else
        {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT)
        {
            phoneXRotate = 90;
        }


        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }


    // Activate Tracking when the program starts.
    public void activateTargets()
    {
        targetsSkyStone.activate();
    }


    // Disable Tracking when we are done.
    public void deactivateTargets()
    {
        targetsSkyStone.deactivate();
    }


    /*
     * This is the primary method we will be using for autonomous navigation.  Vuforia looks for image
     * targets, and if any are found, it updates our absolute location on the field and tells
     * us our x, y, z, roll, pitch, and heading.
     *
     * Note:  allTrackables(0) is the image target on the skystones, which only provide relative location.
     *        This should ONLY be used for target-centric approach.
     */
    public void getLocation()
    {
        // Check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables)
        {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
            {
                master.telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null)
                {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible)
        {
            // Express position (translation) of robot in inches.                                   ***Use this to navigate
            translation = lastLocation.getTranslation();
            master.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // Express the rotation of the robot in degrees.                                        ***Use this to navigate
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            master.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else
        {
            master.telemetry.addData("Visible Target", "none");
        }
    }

    public boolean getTargetVisibility()
    {
        return targetVisible;
    }
}

