package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Rect;

import java.io.File;

@Disabled
@Autonomous(name="OpenCV Test")
public class OpenCVTest extends LinearOpMode
{
    OpenCVDetect findSkystone;
    Dogeforia vuforia;
    WebcamName webcamName;

    // for use once we know focal length -> make sure to check with specs
    public double getHorizontal(double p1, double p2) {
        double middle = findSkystone.width/2;
        double p = (p1 + p2)/2;
        return (p - middle) / (p2 - p1) * 8 ;
    }
    public float findRange(double imageWidth){
        float x = (float) (1/imageWidth);
        return (x - 0.0004f) / 0.0002f;
    }


    public double newFocal(double imageWidth, double imageHeight){
        double add = imageWidth * imageWidth + imageHeight * imageHeight;
        double distance = Math.sqrt(add);
        return distance/2 * 1/Math.tan(39);
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        // instantiate webcam
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // show display on screen
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        // specify Vuforia parameters used to instantiate engine
        parameters.vuforiaLicenseKey = "AQdgAgj/////AAABmZGzg951/0AVjcK/+QiLWG1Z1PfbTwUouhED8hlwM6qrpAncj4xoMYYOUDxF+kreiazigY0q7OMa9XeMyxNlEQvyMFdefVUGSReIxJIXYhFaru/0IzldUlb90OUO3+J4mGvnzrqYMWG1guy00D8EbCTzzl5LAAml+XJQVLbMGrym2ievOij74wabsouyLb2HOab5nxk0FycYqTWGhKmS7/h4Ddd0UtckgnHDjNrMN4jqk0Q9HeTa8rvN3aQpSUToubAmfXe6Jgzdh2zNcxbaNIfVUe/6LXEe23BC5mYkLAFz0WcGZUPs+7oVRQb7ej7jTAJGA6Nvb9QKEa9MOdn0e8edlQfSBRASxfzBU2FIGH8a";
        parameters.fillCameraMonitorViewParent = true;
        parameters.cameraName = webcamName;
        /*parameters.useExtendedTracking = false;*/
        // Instantiate the Vuforia engine
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();
        // create a new OpenCV detector and initialize
        findSkystone = new OpenCVDetect();
        findSkystone.init(hardwareMap.appContext, CameraViewDisplay.getInstance(),0 , true);
        findSkystone.setShowContours(true);
        // have vuforia enabled with the specific detector
        vuforia.setDogeCVDetector(findSkystone);
        vuforia.enableDogeCV();

        //File file = new file(Users/artsy/OneDrive/Desktop/Testing);
        /* Debug only
        // don't show Vuforia vuforia.showDebug();
         */
        // start running Vuforia ( memory and space intensive)
        vuforia.start();

        telemetry.addData("Init:", "complete");
        telemetry.update();

        waitForStart();


        while(opModeIsActive())
        {
            telemetry.addData("Working", "Not Crashed");
            // w * h
            telemetry.addData("Skystone Position:", findSkystone.getSksytonePosition());
            telemetry.addData("total width: ", findSkystone.width);
            telemetry.addData("Distance", findRange(findSkystone.getSksytonePosition().width));
            telemetry.addData("Horizontal",getHorizontal(findSkystone.p1, findSkystone.p2));
            telemetry.addData("P1", findSkystone.p1);
            telemetry.addData("P2", findSkystone.p2);
            telemetry.update();
        }

        findSkystone.disable();
    }
}
