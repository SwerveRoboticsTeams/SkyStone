package org.firstinspires.ftc.team6220_2018;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.ftcrobotcontroller.R;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;


// todo There are a lot of functionalities in this class that may or may not be needed this year
/**
 * This class contains useful methods for integrating vuforia into robot code
 */

public class VuforiaHelper
{
    // Vuforia variables------------------------------------
    VuforiaLocalizer vuforiaLocalizer;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    // Enum that has 4 possible values: UNKNOWN, LEFT, CENTER, and RIGHT
    public RelicRecoveryVuMark vuMark;

    boolean isVuMarkVisible;
    boolean isTracking = false;
    //-------------------------------------------------------

     // What we will eventually return
    float[] colorHSVOutput = new float[]{0, 0, 0};

    int color = 0;
    int colorRGB = 0;
    int avgColorRGB = 0;
    int colorR = 0;
    int colorG = 0;
    int colorB = 0;
    int avgColorR = 0;
    int avgColorG = 0;
    int avgColorB = 0;
    int sumColorR = 0;
    int sumColorG = 0;
    int sumColorB = 0;

    float colorLeft = 0;
    float colorRight = 0;

    float avgLeftJewelColor = 0;
    float avgRightJewelColor = 0;

    // Necessary matrices
    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    Matrix34F rawPose;

    Bitmap bitMap;

    OpenGLMatrix pose;

    // key needed to use vuforia
    public static final String VUFORIA_KEY = "ARnvYoH/////AAAAGQS+OAV8SElJhcYQ4Ud8LkNvhk/zpT8UiiVlkQOGgruNCfQryIqNOyyl6iYhvsCCVYMqHZPJJgORL7ZL3+Hl1VE/CJZiBI357gU4uSmFahasqA9UV/HVmd0Mze0j5cEaVgJ7w3dRhz4Lvdk7qcVwGQTMpAVFKdlpt0657wA0C2vFWzJgZZv3vk7Ouw6bfSltX1/Wgf15jcCcBPRLQs/KkIngbvc+rtBxtD5f4REyb9FuqtN00MoHKL8RIpFQagX/b39JbN8oFLDjUiC5smxchqIHYMIvt7JAQH0TT+fizeIYMnZk3/t8SfNg/gt1lJACY514k9TpM4UwfBvVZcfDVdXj1wKUsPWw8ndUQ6l5PtSq";

    //used for storing info from vuforia frame queue to be analyzed in other methods
    VuforiaLocalizer.CloseableFrame frame;

    //Necessary image variables
    Image image = null;
    int imageFormat;

    public void setupVuforia()
    {
        // Set parameters for VuforiaLocalizer (an interface supporting localization through visual
        // means) and create it
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.useExtendedTracking = false;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // Setup for getting pixel information
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        // Make sure that vuforia doesn't begin racking up unnecessary frames
        vuforiaLocalizer.setFrameQueueCapacity(1);

        // Initialize vision targets; this is important!-----------------
        relicTrackables = this.vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
         // For debugging purposes
        relicTemplate.setName("relicVuMarkTemplate");
        //---------------------------------------------------------------

        /*
         Set phone location on robot. The center of the camera is the origin.
         This location is 90 degrees less than the phone's actual rotation about the z-axis since we
         use the mathematical sense of absolute rotation (0 degrees located on the x-axis) while vuforia
         uses compass coordinates for rotation (0 degrees located on the y-axis)
        */
        // todo Adjust for this year's robot; check y-axis rot fix for phone camera being rotated 180 deg
        phoneLocation = createMatrix(220, 0, 0, -90, 0, 180);

        // Avoids nullpointer errors
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        // Begin tracking targets before match
        startTracking();
    }

    /*
    // function for finding the location of robot
    public OpenGLMatrix getLatestLocation()
    {
        // checks each target to see if we can find our location. If none are visible, then it returns null
        for(int i = 0; i < targets.length; i++)
        {
            // try to find location from this target
            OpenGLMatrix latestLocation = listeners[i].getUpdatedRobotLocation();

            // we've found a target to track
            if(latestLocation != null)
            {
                // tell driver which target is being tracked
                // telemetry.addData("Tracking", targets[i].getName());
                return latestLocation;
            }
        }
        // we've lost track of the targets
        // telemetry.addData("Tracking", "lost");
        return lastKnownLocation;
    }

    //todo change formatting from targets (an array) to relictrackables (an arrayList)
    public void updateLocation()
    {
        // Checks each target to see if we can find our location. If none are visible, then it returns null
        for(int i = 0; i < targets.length; i++)
        {
            // Try to find location from this target
            OpenGLMatrix latestLocation = listeners[i].getUpdatedRobotLocation();

            // We've found a target to track
            if(latestLocation != null)
            {
                lastKnownLocation = latestLocation;
                return;
            }
        }
        // Location is unknown, so don't change anything
    }

    float[] getRobotLocation()
    {
        updateLocation();
        return lastKnownLocation.getTranslation().getData();
    }
    */

    public RelicRecoveryVuMark getVumark()
    {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    //check to make sure we can see the vuMark
    public boolean isVisible()
    {
        if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            isVuMarkVisible = true;
        else
            isVuMarkVisible = false;

        return isVuMarkVisible;
    }


    void startTracking()
    {
        relicTrackables.activate();
        isTracking = true;
    }

    void stopTracking()
    {
        relicTrackables.deactivate();
        isTracking = false;
    }

    // Formats location to something readable
    String format(OpenGLMatrix transformationMatrix)
    {
        return transformationMatrix.formatAsTransform();
    }

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    /*
    // todo Add capability to throw out pixels that are bad (e.g., off jewels or inside jewel holes)
    // This method returns the average RBG of the entire sampling area of one Jewel, then converts the
    // averaged RGB value into an HSV value
    public float getAvgRGBColor(int x, int y)
    {
        // Keeps track of how many "bad" pixels need to be thrown out of the average
        //int badPixelCount = 0;

        sumColorR = 0;
        sumColorG = 0;
        sumColorB = 0;

        // Ensure that we do not attempt to take pixels from outside the image border
        if (x >= Constants.JEWEL_SAMPLE_LENGTH / 2 && x < Constants.IMAGE_WIDTH - Constants.JEWEL_SAMPLE_LENGTH / 2
                && y >= Constants.JEWEL_SAMPLE_HEIGHT / 2 && y < Constants.IMAGE_HEIGHT - Constants.JEWEL_SAMPLE_HEIGHT / 2)
        {
            colorHSVOutput[0] = 0;
            for (int j = y - Constants.JEWEL_SAMPLE_HEIGHT / 2; j < y + Constants.JEWEL_SAMPLE_HEIGHT / 2; j++) // Columns
            {
                for (int i = x - Constants.JEWEL_SAMPLE_LENGTH / 2; i < x + Constants.JEWEL_SAMPLE_LENGTH / 2; i++) // Rows
                {
                    // get RGB color of pixel
                    colorRGB = bitMap.getPixel(i, j);
                    colorR = (colorRGB>>16) & 0x000000FF;
                    colorG = (colorRGB>>8) & 0x000000FF;
                    colorB = colorRGB & 0x000000FF;

                    sumColorR += colorR;
                    sumColorG += colorG;
                    sumColorB += colorB;

                    // Draw white border around sample region for debugging
                    if ((j == y - Constants.JEWEL_SAMPLE_HEIGHT / 2 + 2) || (j == y + Constants.JEWEL_SAMPLE_HEIGHT / 2 - 2)
                            || (i == x - Constants.JEWEL_SAMPLE_LENGTH / 2 + 2) || (i == x + Constants.JEWEL_SAMPLE_LENGTH / 2 - 2))
                    {
                        bitMap.setPixel(i, j, 0xffffffff);
                    }
                }
            }

            // Normalize output for 64x32 = 2048 integration above
            avgColorR = sumColorR / 2048;
            avgColorR = (avgColorR & 0x000000FF) << 16;
            avgColorG = sumColorG / 2048;
            avgColorG = (avgColorG & 0x000000FF) << 8;
            avgColorB = sumColorB / 2048;
            avgColorB = avgColorB & 0x000000FF;
            avgColorRGB = avgColorR + avgColorG + avgColorB;
        }
        // Convert RGB to HSV - hue, saturation, value
        // Hue determines color in a 360 degree circle: 0 red, 60 yellow, 120 green, 180 cyan, 240 blue, 300 magenta
        Color.colorToHSV(avgColorRGB, colorHSVOutput);

        // Return the averaged sampled HSV color value
        return colorHSVOutput[0];
    }
    */
}
