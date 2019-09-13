package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an opmode that tests whether an encoder is working.
 */

@TeleOp(name="TestRunToPosition", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class TestRunToPosition extends LinearOpMode {

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

        runtime.reset(); //reset the object keeping track of the running time of the program

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset the encoder position to 0
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //tell the motor that we want it to run to a target position
        motor.setTargetPosition(2000); //tell it what the target position should be
        motor.setPower(0.25); //tell it the speed it should move at.
        //The power is always positive; the motor will automatically move in the right direction to reach the target

        //isBusy() is true until the motor reaches the target.
        //However... the encoder may be a few ticks +/- the actual target when isBusy becomes false.
        //I have seen the final value be off by as much as 5 ticks, but I don't know what the max can be.
        while (motor.isBusy())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("encoder", "ticks: " + motor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        //go in the other direction, just to show that we can.
        motor.setPower(0);
        motor.setTargetPosition(-2000);
        motor.setPower(0.25);

        while (motor.isBusy()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("encoder", "ticks: " + motor.getCurrentPosition());
            telemetry.update();
            idle();

        }

        motor.setPower(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Done");
            telemetry.addData("encoder", "ticks: " + motor.getCurrentPosition());
            telemetry.update();
            idle();

        }
    }
}
