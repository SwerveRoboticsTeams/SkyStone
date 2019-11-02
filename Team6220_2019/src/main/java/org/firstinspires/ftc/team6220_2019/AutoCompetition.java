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

        // todo May need to change to sideways (180 degrees) for optimal collection
        // Start perpendicular to red wall
        setRobotStartingOrientation(90);

        // Turn on Vuforia tracking
        //vRes.activateTargets();
        waitForStart();
        // Wait to start the match for 0-10 seconds, depending on setup input.
        pauseWhileUpdating(delayCount);

        // Position robot so it can view SkyStone image target
        navigateUsingEncoders(0,16,0.7, false);

        // Find SkyStone image target and translate appropriate distance toward image target
        //vuforiaAlignWithSkyStone();

        // Drive forward and collect SkyStone
        navigateUsingEncoders(0,30,0.5, true);

        // Grab SkyStone and drive backwards (14 in + 3 in behind tile line)
        toggleGrabber();
        navigateUsingEncoders(0,-17,0.5, false);

        // Turn, navigate to center of foundation (+3.5 tiles), and turn again so foundationServos face
        // the foundation
        turnTo(0, 0.7);
        // todo Account for translation shift; done
        navigateUsingEncoders(0,84 - robotShift,0.8, false);
        turnTo(-90, 0.7);

        // Drive up to foundation (forward 3 in + 6 in extra for good measure), activate foundationServos,
        // pull foundation into building site, then deactivate foundationServos
        navigateUsingEncoders(0,-9,0.3, false);
        toggleFoundationServos();
        navigateUsingEncoders(0,38,0.4, false);    // 36 = 2 * 24 - 16 (robot length) + 6 (extra distance)
        toggleFoundationServos();

        // Move lift, drop SkyStone, and retract lift
        runScoringSystemAuto(Constants.LIFT_PLACE_POS);
        toggleGrabber();
        runScoringSystemAuto(Constants.LIFT_GRAB_POS);

        // Navigate two tiles to park on line
        navigateUsingEncoders(56,0,0.8, false);


        // Turn off Vuforia tracking
        //vRes.deactivateTargets();
    }
}
