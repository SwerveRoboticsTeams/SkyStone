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
        waitForStart();
        DcMotor motor = hardwareMap.dcMotor.get("motor");

        motor.setPower(-0.04);
        Thread.sleep(2000);
        motor.setPower(-0.3);
        Thread.sleep(2000);
        motor.setPower(-1.0);
        Thread.sleep(2000);
        motor.setPower(0.0);
    }
}