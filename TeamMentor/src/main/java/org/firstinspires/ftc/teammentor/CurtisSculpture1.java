package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an opmode that moves a motor at a constant rate to power a moving sculpture.
 */

@TeleOp(name="01", group="Curtis")  // @Autonomous(...) is the other common choice
//@Disabled
public class CurtisSculpture1 extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor motor = null;
    //DcMotor rightMotor = null;

    @Override
    public void runOpMode() {

        motor  = hardwareMap.dcMotor.get("motor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        motor.setPower(0.01); //pretty slow

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            idle();

        }
    }
}
