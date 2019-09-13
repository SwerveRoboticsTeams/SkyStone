package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;

//@Autonomous(name="Ground Autonomous", group = "Swerve")
//@Disabled
public class AutoBlue extends MasterAutonomous
{
    private OpenCVDetect goldVision;
    boolean isLeftGold = false;
    boolean isCenterGold = false;
    boolean isRightGold = false;

    public void runOpMode() throws InterruptedException
    {
        autoInitializeRobot();

        goldVision = new OpenCVDetect();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        goldVision.setShowCountours(false);
        // start the vision system
        goldVision.enable();
        telemetry.addData("Done: ", "initializing");
        telemetry.update();

        while (!isStarted())
        {
            // select position left or right, from drivers facing the field
            if (gamepad1.x) isPosCrater = true;
            if (gamepad1.b) isPosCrater = false;

            if (isPosCrater) telemetry.addData("Alliance: ", "Crater");
            else telemetry.addData("Alliance: ", "Depot");
            // sample
            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2), (goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2))
            );

            if (((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) == 0) && ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) == 0)) {
                isLeftGold = false;
                isCenterGold = false;
                isRightGold = true;
                telemetry.addLine("Right");
            } else if (((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) <= 200) /*&& ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 470)*/) {
                isLeftGold = true;
                isCenterGold = false;
                isRightGold = false;
                telemetry.addLine("Left");
            } else if (((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) >= 400) /*&& ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 470)*/) {
                isLeftGold = false;
                isCenterGold = true;
                isRightGold = false;
                telemetry.addLine("Center");
            }
            telemetry.update();
            idle();
        }
        telemetry.update();
        //marker.setPosition(MARKER_LOW);
        waitForStart();
        autoRuntime.reset();
        goldVision.disable();
        //autoRuntime.reset();
        telemetry.addData("Auto: ", "Started");
        telemetry.update();

        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

        if (!isPosCrater)
        {
            if (isRightGold)
            {
                pivotWithReference(55, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                sleep(100); // pause 100 milliseconds
                move(0, 550, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(100); // pause 100 milliseconds
                pivotWithReference(135, refAngle, 0.2, 0.75); // turn to align
                sleep(100); // pause 100 milliseconds
                move(0, 400, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(100);
                pivotWithReference(50, refAngle, 0.2, 0.75); // turn so pusher is facing the crater
                //marker.setPosition(MARKER_HIGH);
                sleep(100);
                move(-170, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                move(0, -2700, 0.3, 0.7, 3.0);// park in blue crater
                //marker.setPosition(MARKER_LOW);
            }
            else if (isLeftGold)
            {
                pivotWithReference(115, refAngle, 0.2, 0.75);
                sleep(100);
                move(0, 570, 0.3, 0.7, 2.0); // push the gold mineral
                sleep(100);
                pivotWithReference(45, refAngle, 0.2, 0.75); // turn so pusher is facing the crater
                sleep(100);
                move(0, 500, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(100);
                //marker.setPosition(MARKER_HIGH);
                pivotWithReference(50, refAngle, 0.2, 0.75); // turn to align
                sleep(100);
                move(-200, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                move(0, -2550, 0.3, 0.7, 3.0);// park in blue crater
                //marker.setPosition(MARKER_LOW);
            }
            else if (isCenterGold)
            {
                pivotWithReference(87, refAngle, 0.2, 0.5); // turn to face the sampling field
                sleep(100);
                move(0, 800, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(100);
                pivotWithReference(50, refAngle, 0.2, 0.75); // turn to align
                sleep(100);
                //marker.setPosition(MARKER_HIGH);
                move(-180, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                move(-8, -2600, 0.3, 0.7, 3.0);// park in blue crater
                //marker.setPosition(MARKER_LOW);
            }
        }

        if (isPosCrater)
        {
            if (isLeftGold)
            {
                pivotWithReference(115, refAngle, 0.2, 0.75);
                sleep(100);
                move(0, 500, 0.3, 0.7, 2.0); // push the gold mineral
            }
            else if (isRightGold)
            {
                pivotWithReference(55, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                sleep(100);
                move(0, 500, 0.3, 0.7, 3.0); // push the gold mineral
            }
            else if (isCenterGold)
            {
                pivotWithReference(87, refAngle, 0.2, 0.5); // turn to face the sampling field
                sleep(100);
                move(0, 500, 0.3, 0.7, 3.0); // push the gold mineral
            }

            moveTimed(0.55, 1200); // push gold
            sleep(100);
            pivotWithReference(0, refAngle, 0.2, 0.75); // face crater
            sleep(200);
            moveTimed(0.55, 500); // go into crater / park
        }

    }
}

