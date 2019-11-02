package org.firstinspires.ftc.team6220_2019;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

/**
 * General autonomous class.  During initialization, driver 1 selects various options, including
 * alliance, delay time, number of SkyStones, whether we want to soore the foundation, and whether
 * we want to park close or far under the Skybridge.  These options allow our robot to adapt to the
 * preferences of various alliance partners.
 */
// todo Autonomous code is currently untested.

@Autonomous(name = "AutoCompetition")
public class AutoCompetition extends MasterAutonomous
{

    public int[][] navPoints = {{-18, -18},
                                {-18, 18},
                                {-18, 18},
                                {-18, -18}};

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        // todo May need to change to sideways for optimal collection
        // Start perpendicular to red wall
        setRobotStartingOrientation(90);

        // Turn on Vuforia tracking
        vRes.activateTargets();
        waitForStart();
        // Wait to start the match for 0-10 seconds, depending on setup input.
        pauseWhileUpdating(delayCount);

        // Position robot so it can view SkyStone image target
        navigateUsingEncoders(0,16,0.7, false);

        // Find SkyStone image target

        // Translate appropriate distance toward image target

        // Drive forward and collect SkyStone
        navigateUsingEncoders(0,28,0.5, true);

        // Grab SkyStone and drive backwards (12 in + 3 in behind tile line)
        toggleGrabber();
        navigateUsingEncoders(0,-15,0.5, false);

        // Turn, navigate to center of foundation (+3.5 tiles), and turn again so foundationServos face
        // the foundation
        turnTo(0, 0.7);
        // todo Account for translation shift
        navigateUsingEncoders(0,84,0.8, false);
        turnTo(-90, 0.7);

        // Drive up to foundation (forward 3 in + 1 in extra for good measure), activate foundationServos,
        // pull foundation into building site, then deactivate foundationServos
        navigateUsingEncoders(0,-4,0.3, false);
        toggleFoundationServos();
        navigateUsingEncoders(0,33,0.5, false);    // 33 = 2 * 24 - 16 (robot length) + 1 (extra distance)
        toggleFoundationServos();

        // Move lift, drop SkyStone, and retract lift
        liftMotor.setTargetPosition(Constants.LIFT_PLACE_POS);  // todo Adjust LIFT_PLACE_POS
        liftMotor.setPower(Constants.LIFT_POWER_FACTOR);
        pauseWhileUpdating(1.5);    // todo Adjust time
        toggleGrabber();
        liftMotor.setTargetPosition(Constants.LIFT_GRAB_POS);
        liftMotor.setPower(Constants.LIFT_POWER_FACTOR);
        pauseWhileUpdating(1.5);

        // Navigate two tiles to park on line
        navigateUsingEncoders(48,0,0.8, false);


        // Turn off Vuforia tracking
        vRes.deactivateTargets();
    }
}
