package org.firstinspires.ftc.team6220_2019.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "DBTest", group = "TeleOp")
public class DBTest extends LinearOpMode {

    //Motors
    DcMotor motorLeft;
    DcMotor motorRight;

    //Double



    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorLeft.setTargetPosition(0);
        //motorLeft.setTargetPosition(0);
        //motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive())
        {
            //motorLeft.setPower( (4/5) * (0.5 * Math.pow(gamepad1.left_stick_y, 3)) + (0.5 * gamepad1.left_stick_y));
            //motorRight.setPower( (4/5) * (0.5 * Math.pow(gamepad1.right_stick_y, 3)) + (0.5 * gamepad1.right_stick_y));

            if(gamepad1.a)
            {
                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);

                motorLeft.setTargetPosition(1120 * 1);
                motorRight.setTargetPosition(1120 * 1);
                motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            //Telemetry
            telemetry.addData("MotorLeftPower", -motorLeft.getPower());
            telemetry.addData("MotorRightPower", -motorRight.getPower());
            telemetry.addData("EncoderLeft", motorLeft.getCurrentPosition());
            telemetry.addData("EncoderRight", motorRight.getCurrentPosition());
            telemetry.update();


            idle();
        }
    }
}
