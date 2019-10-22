package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor Test")
public class MotorTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        DcMotor motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);

        waitForStart();

        while (opModeIsActive())
        {
            motor.setPower(gamepad1.right_stick_y);

            telemetry.addData("Motor Power: ", motor.getPower());
            telemetry.addData("Motor Position: ", motor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}