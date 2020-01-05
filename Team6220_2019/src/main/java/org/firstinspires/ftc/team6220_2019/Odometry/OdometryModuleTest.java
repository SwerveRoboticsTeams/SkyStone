package org.firstinspires.ftc.team6220_2019.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class is meant for testing the functionality of the encoder on an odometry module attached to a single REV hub.
 * It is not meant to be used to test odometry navigation.
 */
@TeleOp(name = "Odometry Module Test")
public class OdometryModuleTest extends LinearOpMode
{
    public void runOpMode() throws InterruptedException
    {
        // Initialize encoders
        DcMotor odo1 = hardwareMap.dcMotor.get("odo1");
        odo1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Position: ", odo1.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
