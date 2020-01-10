package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * General autonomous class.  During initialization, driver 1 selects various options, including
 * alliance, delay time, number of SkyStones, whether we want to soore the foundation, and whether
 * we want to park close or far under the Skybridge.  These options allow our robot to adapt to the
 * preferences of various alliance partners.
 */
// todo Test old auto code to see if it works.
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

        /* Outline of autonomous (written from perspective of red team)
        Be able to start at two positions. One with x = -36in, one with x = 24in.

        If alliance partner is doing autonomous, translate left two feet.

        Else, translate forward by 12in. Pause briefly to determine which stone is skystone.
        Rotate robot 180 degrees. --FROM NOW ON, ALL INSTRUCTIONS GIVEN WITH NEW ORIENTATION--
        Align robot by translating left/right for skystone. Translate forward around two feet to collect
        skystone. Ensure that the position of the arm is ready to collect stone.

        Once collected, grabber should close. Translate backwards to one of two positions, depending on
        whether we are using inside lane or outside lane (depends on where alliance robot is). Translate right until
        center of robot is around 24in from right edge.

        Translate forward an appropriate amount (depending on which lane used), then lower foundation servos.
        Drive backwards until the back of the robot is against the wall. Extend arm, drop grabber.

        Translate left to get out of the corner (possibly engaging collector motors to counteract the
        inevitable friction), then translate to the designated parking spot (depends on where
        alliance partner is). 
         */


        // todo Currently don't need trackables; will probably rely on odometry rather than Vuforia.
        // Turn off Vuforia tracking.
        //vuf.deactivateTargets();
        // Turn off OpenCV.
        //skystoneDetector.disable();
    }
}
