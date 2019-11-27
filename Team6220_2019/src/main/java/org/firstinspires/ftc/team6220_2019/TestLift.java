package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class for powering lift motors using gamepad input.
 */

@TeleOp(name = "Test Lift")
public class TestLift extends MasterOpMode
{
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;

    CRServo vex1;
    CRServo vex2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");

        vex1 = hardwareMap.crservo.get("vex1");
        vex2 = hardwareMap.crservo.get("vex2");

        waitForStart();

        while (opModeIsActive())
        {
            //motor1.setPower(gamepad1.right_stick_y);
            //motor2.setPower(gamepad1.right_stick_y);
            //motor3.setPower(gamepad1.right_stick_y);
            vex1.setPower(0.85 * gamepad1.left_stick_y);
            vex2.setPower(-0.85 * gamepad1.left_stick_y);

            telemetry.addData("Power1:", vex1.getPower());
            telemetry.addData("Power2:", vex2.getPower());
            telemetry.update();
            idle();
        }
    }
}
