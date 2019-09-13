package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an opmode that tests whether an encoder is working.
 */

@TeleOp(name="TestMotor", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class TestMotor extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor = null;
    //DcMotor rightMotor = null;

    @Override
    public void runOpMode() {

        motor  = hardwareMap.dcMotor.get("motor");

        //using an AndyMark motor so we need to reverse its direction
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Power: " + gamepad1.left_stick_y);
            telemetry.update();
            idle();

        }
    }
}
