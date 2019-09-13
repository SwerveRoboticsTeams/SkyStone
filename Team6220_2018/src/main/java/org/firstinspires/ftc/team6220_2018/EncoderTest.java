package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Program to test functionality of encoders
 */

@Autonomous(name = "Encoder Test")
public class EncoderTest extends MasterOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeRobot();

        waitForStart();

        while (opModeIsActive())
        {
            motorBR.setPower(1.0);
            motorBL.setPower(1.0);
            motorFR.setPower(1.0);
            motorFL.setPower(1.0);

            telemetry.addData("FL enc value ", motorFL.getCurrentPosition());
            telemetry.addData("FR enc value ", motorFR.getCurrentPosition());
            telemetry.addData("BL enc value ", motorBL.getCurrentPosition());
            telemetry.addData("BR enc value ", motorBR.getCurrentPosition());
            telemetry.update();
        }
    }
}
