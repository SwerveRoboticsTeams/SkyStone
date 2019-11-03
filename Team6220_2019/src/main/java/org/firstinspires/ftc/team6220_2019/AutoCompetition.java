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
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        // Start perpendicular to red wall
        setRobotStartingOrientation(initAngle_side);

        // Turn on Vuforia tracking
        vRes.activateTargets();
        waitForStart();
        // Wait to start the match for 0-10 seconds, depending on setup input.
        pauseWhileUpdating(delayCount);

        // Position robot so it can view SkyStone image target, wait for a bit to recognize target
        navigateUsingEncoders(0,12,0.4, false);
        pauseWhileUpdating(3.0);

        // Find SkyStone image target and translate appropriate distance toward image target
        vuforiaAlignWithSkyStone();

        // Drive forward and collect SkyStone
        navigateUsingEncoders(0,36,0.2, true);

        // Lower grabber, then grab SkyStone and drive backwards (18 in + 3 in behind tile line)
        runScoringSystemAuto(0);
        toggleGrabber();
        navigateUsingEncoders(0,-22,0.5, false);

        // todo Need to add if() statement to distinguish whether we are scoring foundation

        // Turn, navigate to center of foundation (+3.5 tiles), and turn again so foundationServos face
        // the foundation
        turnTo(0, 0.7);
        navigateUsingEncoders(0,84 - robotShiftSign * robotShift - centerAdjustment,0.7, false);
        turnTo(-90 + turnShift, 0.7);   // Account for turn shift if blue alliance (+180)

        // Drive up to foundation (forward 3 in + 6 in extra for good measure), activate foundationServos,
        // pull foundation into building site, then deactivate foundationServos
        navigateUsingEncoders(0,-9,0.3, false);
        toggleFoundationServos();
        // todo Need to adjust backward translation?
        navigateUsingEncoders(0,38,0.4, false);    // 38 = 2 * 24 - 16 (robot length) + 6 (extra distance)
        // todo Need turnTo() to move foundation into building site?
        toggleFoundationServos();

        // Move lift, drop SkyStone, and retract lift
        runScoringSystemAuto(Constants.LIFT_PLACE_POS);
        pauseWhileUpdating(0.5);
        toggleGrabber();
        runScoringSystemAuto(Constants.LIFT_GRAB_POS);

        // Navigate two tiles to park on line
        navigateUsingEncoders(robotShiftSign * 56,0,0.7, false);


        // Turn off Vuforia tracking
        vRes.deactivateTargets();
    }
}
