package org.firstinspires.ftc.team417_2018;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

//@Autonomous(name="Sample for Alliance", group = "Swerve")
// @Disabled
public class AutoHangSampleforAlliance extends MasterAutonomous
{
    public void runOpMode() throws InterruptedException
    {
        autoInitializeRobot();
        InitializeDetection();

        while (!isStarted())
        {
            // select position left or right, from drivers facing the field
            if (gamepad1.x) isPosCrater = true;
            if (gamepad1.b) isPosCrater = false;

            if (-gamepad1.right_stick_y > 0)
            {
                threshold ++;
                sleep(100);
            }
            if (-gamepad1.right_stick_y < 0)
            {
                threshold --;
                sleep(100);
            }
            OpenCV_detector.setThreshold(threshold);
            telemetry.addData("threshold", threshold);

            if (isPosCrater) telemetry.addData("Alliance: ", "CRATER");
            else telemetry.addData("Alliance: ", "DEPOT");

            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2), (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2)));
            locateGold();

            telemetry.update();
        }
        waitForStart();
        autoRuntime.reset();
        telemetry.addData("Auto: ", "Started");

        land();
        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

        move(45, 0, 0.2, 0.75, 3.0); // move robot hanger off the lander hook
        sleep(50);

        move(0, 70, 0.2, 0.75, 3.0); // move from sampling position to gold push position
        sleep(50);

        if (!isPosCrater) // if it's the depot option
        {
            if (isLeftGold)
            {
                pivotWithReference(35, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                sleep(100); // pause 100 milliseconds
                move(0, 450, 0.3, 0.7, 3.0); // go forward to push the gold mineral
                sleep(100); // pause 100 milliseconds
                pivotWithReference(-45, refAngle, 0.2, 0.75); // turn to face depot
                sleep(100); // pause 100 milliseconds
                move(-70, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                move(0, 400, 0.3, 0.7, 3.0); // go towards depot
                sleep(100);
                marker.setPosition(MARKER_HIGH); // drop the team marker
                sleep(100);
                move(0, -250, 0.3, 0.7, 2.0); // back up
                sleep(100);
                move(-170, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                // for not sampling with alliance
                move(0, -600, 0.3, 0.7, 3.0); // park in blue crater
                marker.setPosition(MARKER_LOW); // lower marker arm
                // for sampling for alliance
                move(0, -400, 0.3, 0.7, 3.0);
                pivotWithReference(120, refAngle, 0.2, 0.75);
                move(0, 300, 0.3, 0.7, 3.0);
                pivotWithReference(80, refAngle, 0.2, 0.75);
                move(0, 400, 0.3, 0.7, 3.0);

            }
            else if (isRightGold)
            {
                pivotWithReference(-28, refAngle, 0.2, 0.75); // pivot to face gold mineral
                sleep(100);
                move(0, 420, 0.3, 0.7, 2.0); // go forward to push the gold mineral
                sleep(100);
                pivotWithReference(45, refAngle, 0.2, 0.75); // turn to face the crater
                sleep(100);
                move(120, 0, 0.3, 0.7, 3.0); // move closer to depot wall
                sleep(100);
                move(0, 400, 0.3, 0.7, 3.0); // drive into depot
                sleep(100);
                pivotWithReference(-45, refAngle, 0.2, 0.75); // turn to face wall opposite to crater
                sleep(100);
                marker.setPosition(MARKER_HIGH); // drop marker in depot
                sleep(100);
                move(0, -150, 0.3, 0.7, 3.0); // move out of depot
                sleep(100);
                move(-150, 0, 0.3, 0.7, 2.0); // avoid left mineral
                sleep(100);
                // for sampling for alliance
                move(0, -400, 0.3, 0.7, 3.0);
                sleep(100);
                pivotWithReference(120, refAngle, 0.2, 0.75);
                sleep(100);
                move(0, 300, 0.3, 0.7, 3.0);
                sleep(100);
                pivotWithReference(80, refAngle, 0.2, 0.75);
                sleep(100);
                move(0, 400, 0.3, 0.7, 3.0);
            }
            else if (isCenterGold)
            {
                move(0, 700, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(100);
                pivotWithReference(-45, refAngle, 0.2, 0.75); // turn to face depot
                sleep(100);
                marker.setPosition(MARKER_HIGH); // drop the marker in the depot
                sleep(100);
                move(0, -100, 0.3, 0.7, 2.0); // back up
                sleep(100);
                move(-250, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                marker.setPosition(MARKER_LOW); // lower marker arm
                // for sampling for alliance
                move(0, -400, 0.3, 0.7, 3.0);
                sleep(100);
                pivotWithReference(120, refAngle, 0.2, 0.75);
                sleep(100);
                move(0, 300, 0.3, 0.7, 3.0);
                sleep(100);
                pivotWithReference(80, refAngle, 0.2, 0.75);
                sleep(100);
                move(0, 400, 0.3, 0.7, 3.0);

            }
        }
        if (isPosCrater)
        {
            if (isRightGold)
            {
                pivotWithReference(-25, refAngle, 0.2, 0.75); // pivot to face gold mineral
                sleep(100);
                move(0, 400, 0.3, 0.7, 2.0); // go forward to push the gold mineral
                sleep(100);
                // rest of this code is for dropping of marker after knocking gold mineral
                sleep(100);
                move(0, -130, 0.3, 0.7, 2.0); // back up from the gold mineral
                sleep(100);
                pivotWithReference(90, refAngle, 0.2, 0.75); // pivot to face wall
                sleep(100);
                move(-30, 0, 0.3, 0.7, 2.0); // go forwards toward the wall
                sleep(100);
                move(0, 700, 0.3, 0.7, 2.0); // go forwards toward the wall
                sleep(100);
                pivotWithReference(135, refAngle, 0.2, 0.75); // pivot to face the depot
                sleep(100);
                move(100, 0, 0.3, 0.7, 2.0); // translate towards the wall
                sleep(50);
                move(0, 550, 0.3, 0.7, 2.0); // go forwards into the depot
                sleep(50);
                marker.setPosition(MARKER_HIGH); // release the TM
                sleep(100);
                pivotWithReference(135, refAngle, 0.2, 0.75); // pivot to face the depot
                sleep(100);
                move(0,-400,0.3,0.72,3.5); // go backwards to park in the crater
                sleep(100);
                move(250,0,0.3,0.72,3.5); // go backwards to park in the crater
                sleep(100);
                move(-400,0,0.3,0.72,3.5); // go backwards to park in the crater
                marker.setPosition(MARKER_LOW); // lower the TMD
               sleep(100);
                move(0,-400,0.3,0.72,3.5); // park in the crater
            }
            else if (isLeftGold)
            {
                pivotWithReference(33, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                sleep(100);
                move(0, 400, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(100);
                move(0, -130, 0.3, 0.7, 2.0); // back up from the gold mineral
                sleep(100);
                pivotWithReference(90, refAngle, 0.2, 0.75); // pivot to face wall
                sleep(100);
                move(0, 410, 0.3, 0.7, 2.0); // go forwards toward the wall
                sleep(100);
                pivotWithReference(135, refAngle, 0.2, 0.75); // pivot to face the depot
                sleep(100);
                move(85, 0, 0.3, 0.7, 2.0); // translate towards the wall
                sleep(50);
                move(0, 600,0.3, 0.7, 2.0); // go forwards into the depot
                sleep(50);
                marker.setPosition(MARKER_HIGH); // release the TM
                sleep(100);
                move(0,-0,0.3,0.72,3.5); // go backwards to park in the crater
                sleep(100);
                move(250,0,0.3,0.72,3.5); // go backwards to park in the crater
                sleep(100);
                move(-400,0,0.3,0.72,3.5); // go backwards to park in the crater
                marker.setPosition(MARKER_LOW); // lower the TMD
                sleep(100);
                move(0,-700,0.3,0.72,3.5); // park in the crater
            }
            else if (isCenterGold)
            {
                sleep(100);
                move(0, 350, 0.3, 0.7, 3.0); // push the gold mineral
                // rest of this code is for dropping of marker after knocking gold mineral
                sleep(100);
                move(0, -130, 0.3, 0.7, 2.0); // back up from the gold mineral
                sleep(100);
                pivotWithReference(90, refAngle, 0.2, 0.75); // pivot to face wall
                sleep(100);
                move(0, 610, 0.3, 0.7, 2.0); // go forwards toward the wall
                sleep(100);
                pivotWithReference(135, refAngle, 0.2, 0.75); // pivot to face the depot
                sleep(100);
                move(85, 0, 0.3, 0.7, 2.0); // translate towards the wall
                sleep(50);
                move(0, 550, 0.3, 0.7, 2.0); // go forwards into the depot
                sleep(50);
                marker.setPosition(MARKER_HIGH); // release the TM
                move(0,-650,0.3,0.72,3.5); // go backwards to park in the crater
                marker.setPosition(MARKER_LOW); // lower the TMD
                pivotWithReference(-40, refAngle, 0.2, 0.75); // pivot to face the crater
                sleep(100);
                move(0,200,0.3,0.72,3.5); // park in the crater
            }
        }
        rev1.setPosition(0.0);
        lowerArm();
        vuforia.stop();
    }
}

