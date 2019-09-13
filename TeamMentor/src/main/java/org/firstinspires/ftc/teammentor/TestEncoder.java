package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an opmode that tests whether an encoder is working.
 */

@TeleOp(name="TestEncoder", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class TestEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor = null;
    //DcMotor rightMotor = null;

    @Override
    public void runOpMode() {

        motor  = hardwareMap.dcMotor.get("motor");

        //using an AndyMark motor so we need to reverse its direction
        //No longer need to reverse it since we can declare the motor as an andymark in our config
        //motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //motor.setMaxSpeed(100);
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("encoder", "ticks: " + motor.getCurrentPosition());
            telemetry.update();
            idle();

        }
    }
}
