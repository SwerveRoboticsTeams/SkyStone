package org.firstinspires.ftc.team417_2019.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_2019.MasterAutonomous;

import java.io.IOException;

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


        //pivot(180, 0.1, 1.0);


        pivot(-180, 0.8);

    }

        //move(0, 24,  0.7);
        //move(24, 24,  0.7);
        //move(0, -24, 0.6);
        //move(24, 24, 0.6);
}
