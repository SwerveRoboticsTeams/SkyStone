package org.firstinspires.ftc.team417_2019.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        goToPosition2(48, 0,  0.2);
        //goToPosition2(0, 24, 0.8);
    }
}
