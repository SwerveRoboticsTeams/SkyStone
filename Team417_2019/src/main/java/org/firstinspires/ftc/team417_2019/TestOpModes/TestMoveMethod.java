package org.firstinspires.ftc.team417_2019.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_2019.MasterAutonomous;

//@Disabled
@Autonomous(name = "Test Move")
public class TestMoveMethod extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoInitializeRobot();

        // change depending on where robot starts
        robot.setCorrectedHeading(0);

        telemetry.update();

        waitForStart();


        pivot(180, 0, 0.9);
        /*
        pause(1000);
        pivot(0, 0, 0.6);
        pause(1000);
        pivot(45, 0, 0.7);
        pause(1000);
        pivot(-90, 0, 0.4);
        pause(1000);
        pivot(73, 0, 0.2);

         */

        //goToPosition2(0, 24,  0.5);
        //goToPosition2(0, 24, 0.8);
    }
}
