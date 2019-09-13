package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

@Autonomous(name="Hang Autonomous", group = "Swerve")
// @Disabled
public class AutoHang extends MasterAutonomous
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
            if (gamepad1.dpad_up)
            {
                delay += 1000;
                sleep(100);
            }
            if (gamepad1.dpad_down)
            {
                delay -= 1000;
                sleep(100);
            }

            if (-gamepad1.right_stick_y > 0)
            {
                threshold++;
                sleep(100);
            }
            if (-gamepad1.right_stick_y < 0)
            {
                threshold--;
                sleep(100);
            }
            OpenCV_detector.setThreshold(threshold);
            telemetry.addData("threshold", threshold);
            telemetry.addData("delay", delay);

            if (isPosCrater) telemetry.addData("Alliance: ", "CRATER");
            else telemetry.addData("Alliance: ", "DEPOT");

            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2), (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2)));
            locateGold();

            telemetry.update();
        }
        waitForStart();

        autoRuntime.reset();
        vuforia.stop();
        telemetry.addData("Auto: ", "Started");

        landEncoder(2000, 2220);
        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

        move(47, 0, 0.2, 0.75, 3.0); // move robot hanger off the lander hook
        sleep(50);

        if (!isPosCrater) // if it's the depot option
        {
            if (goldLocation == "LEFT")
            {
                move(0, 10, 0.3, 0.7, 1.0); // move away from the lander
                sleep(40);
                pivotWithReference(35, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                sleep(50); // pause 100 milliseconds
                move(0, 450, 0.3, 0.7, 3.0); // go forward to push the gold mineral
                sleep(50); // pause 100 milliseconds
                pivotWithReference(-45, refAngle, 0.2, 0.75); // turn to face depot
                sleep(50); // pause 100 milliseconds
                move(-90, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(50);
                move(0, 400, 0.3, 0.7, 3.0); // go towards depot
                sleep(50);
                marker.setPosition(MARKER_HIGH); // drop the team marker
                sleep(50);
                move(0,-80,0.3,0.75,2.0);
                sleep(50);
                pivotWithReference(135, refAngle,0.3,0.75);
                sleep(50);
                move(250, 0, 0.3, 0.7, 2.0); // align against wall
                sleep(50);
                move(0, 610, 0.3, 0.7, 3.0); // park in blue crater
                marker.setPosition(MARKER_LOW); // lower marker arm
            }
            else if (goldLocation == "RIGHT")
            {
                move(0, 10, 0.3, 0.7, 1.0); // move away from the lander
                sleep(40);
                pivotWithReference(-26, refAngle, 0.2, 0.75); // pivot to face gold mineral
                sleep(50);
                move(0, 420, 0.3, 0.7, 2.0); // go forward to push the gold mineral
                sleep(50);
                pivotWithReference(45, refAngle, 0.2, 0.75); // turn to face the crater
                sleep(50);
                move(120, 0, 0.3, 0.7, 3.0); // move closer to depot wall
                sleep(50);
                move(0, 400, 0.3, 0.7, 3.0); // drive into depot
                sleep(50);
                pivotWithReference(-45, refAngle, 0.2, 0.75); // turn to face wall opposite to crater
                sleep(50);
                marker.setPosition(MARKER_HIGH); // drop marker in depot
                sleep(50);
                move(0, -100, 0.3, 0.7, 3.0); // move out of depot
                sleep(50);
                pivotWithReference(135,refAngle,0.2,0.75);
                move(170, 0, 0.3, 0.7, 2.0); // avoid left mineral
                sleep(50);
                move(0, 620, 0.3, 0.7, 3.0); // park in opposite crater
                marker.setPosition(MARKER_LOW); // lower marker arm
            }
            else if (goldLocation == "CENTER")
            {
                move(0, 700, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(50);
                marker.setPosition(MARKER_HIGH); // drop the marker in the depot
                sleep(50);
                move(0, -100, 0.3, 0.7, 2.0); // back up
                pivotWithReference(135, refAngle,0.3,0.75);
                move(300, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(50);
                move(0, 650, 0.3, 0.7, 3.0); // park in blue crater
                sleep(50);
                marker.setPosition(MARKER_LOW); // lower marker arm
            }
        }
        if (isPosCrater)
        {
            if (goldLocation == "RIGHT")
            {
                move(0, 10, 0.3, 0.7, 1.0); // move away from the lander
                sleep(40);
                pivotWithReference(-27, refAngle, 0.2, 0.75); // pivot to face gold mineral
                sleep(50);
                move(0, 355, 0.3, 0.7, 2.0); // go forward to push the gold mineral
                sleep(50);
                // rest of this code is for dropping of marker after knocking gold mineral
                sleep(50);
                move(0, -105, 0.3, 0.7, 2.0); // back up from the gold mineral
                sleep(50);
                sleep(delay);
                pivotWithReference(90, refAngle, 0.2, 0.75); // pivot to face wall
                sleep(50);
                //move(-30, 0, 0.3, 0.7, 2.0); // go sideways toward the wall
                sleep(50);
                move(0, 700, 0.3, 0.7, 2.0); // go forwards toward the wall
                sleep(50);
                pivotWithReference(135, refAngle, 0.2, 0.75); // pivot to face the depot
                sleep(50);
                //move(100, 0, 0.3, 0.7, 2.0); // translate towards the wall
                sleep(50);
                move(0, 550, 0.3, 0.7, 2.0); // go forwards into the depot
                sleep(50);
                move(150, 0, 0.3, 0.7, 2.0); // translate towards the wall
                sleep(50);
                marker.setPosition(MARKER_HIGH); // release the TM
                sleep(50);
                move(0,-700,0.3,0.72,3.5); // go backwards to park in the crater
                marker.setPosition(MARKER_LOW); // lower the TMD
                pivotWithReference(-40, refAngle, 0.2, 0.75); // pivot to face the crater
                move(0,75,0.3,0.72,3.5); // park in the crater
                /*
                pivotWithReference(0, refAngle, 0.2, 0.75); // face crater
                sleep(200);
                moveTimed(0.55, 500); // go into crater / park
                */
            }
            else if (goldLocation == "LEFT")
            {
                move(0, 10, 0.3, 0.7, 1.0); // move away from the lander
                sleep(40);
                pivotWithReference(33, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                sleep(50);
                move(0, 400, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(50);
                move(0, -130, 0.3, 0.7, 2.0); // back up from the gold mineral
                sleep(50);
                sleep(delay);
                pivotWithReference(90, refAngle, 0.2, 0.75); // pivot to face wall
                sleep(50);
                move(0, 410, 0.3, 0.7, 2.0); // go forwards toward the wall
                sleep(50);
                pivotWithReference(135, refAngle, 0.2, 0.75); // pivot to face the depot
                sleep(50);
                move(85, 0, 0.3, 0.7, 2.0); // translate towards the wall
                sleep(50);
                move(0, 600,0.3, 0.7, 2.0); // go forwards into the depot
                sleep(50);
                marker.setPosition(MARKER_HIGH); // release the TM
                sleep(50);
                //pivotWithReference(135, refAngle, 0.2, 0.75); // pivot so back of robot faces our crater
                move(0,-700,0.3,0.72,3.5); // go backwards to park in the crater
                marker.setPosition(MARKER_LOW); // lower the TMD
                pivotWithReference(-40, refAngle, 0.2, 0.75); // pivot to face the crater
                move(0,200,0.3,0.72,3.5); // park in the crater
                /*
                sleep(100);
                pivotWithReference(0, refAngle, 0.2, 0.75); // face crater
                sleep(200);
                moveTimed(0.55, 500); // go into crater and park
                */
            }
            else if (goldLocation == "CENTER")
            {
                sleep(50);
                move(0, 360, 0.3, 0.7, 3.0); // push the gold mineral

                // rest of this code is for dropping of marker after knocking gold mineral
                sleep(50);
                move(0, -130, 0.3, 0.7, 2.0); // back up from the gold mineral
                sleep(50);
                sleep(delay);
                pivotWithReference(90, refAngle, 0.35, 0.75); // pivot to face wall
                sleep(50);
                move(0, 610, 0.3, 0.7, 2.0); // go forwards toward the wall
                sleep(50);
                pivotWithReference(135, refAngle, 0.35, 0.75); // pivot to face the depot
                sleep(50);
                move(85, 0, 0.3, 0.7, 2.0); // translate towards the wall
                sleep(50);
                move(0, 550, 0.3, 0.7, 2.0); // go forwards into the depot
                sleep(50);
                marker.setPosition(MARKER_HIGH); // release the TM
                move(0,-650,0.3,0.72,3.5); // go backwards to park in the crater
                marker.setPosition(MARKER_LOW); // lower the TMD
                sleep(50);
                pivotWithReference(-40, refAngle, 0.35, 0.75); // pivot to face the crater
                sleep(50);
                move(0,200,0.3,0.72,3.5); // park in the crater
            }
        }

        //autoExtendSlides();
        }
    }

