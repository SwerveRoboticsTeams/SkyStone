package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * General autonomous class.  During initialization, driver 1 selects various options, including
 * alliance, delay time, number of SkyStones, whether we want to soore the foundation, and whether
 * we want to park close or far under the Skybridge.  These options allow our robot to adapt to the
 * preferences of various alliance partners.
 */
// todo Need to clean up implementation of buffet autonomous.

@Autonomous(name = "AutoCompetition")
public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize(true, true);

        // Start perpendicular to red wall
        setRobotStartingOrientation(initAngle_side);


        waitForStart();
        // Wait to start the match for 0-10 seconds, depending on setup input.
        pauseWhileUpdating(delayCount);

        // todo Add new auto code!

        // Turn off Vuforia tracking
        vuf.deactivateTargets();
    }
}
