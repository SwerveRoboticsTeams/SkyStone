package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;

abstract class MasterTeleOp extends Master
{


    boolean isRunToPosMode = false;

    public void driveMecanumTeleOp()
    {
        // Reverse drive if desired
        if(gamepad1.right_stick_button)
            reverseDrive = false;
        if(gamepad1.left_stick_button)
            reverseDrive = true;

        if(gamepad1.dpad_down)
            slowModeDivisor = 3.0;
        else if(gamepad1.dpad_up)
            slowModeDivisor = 1.0;

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = 0.5 * gamepad1.left_stick_x;
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation
        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveMecanum(angle, power, turnPower);
    }

    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    public void runIntake()
    {
        if (gamepad1.dpad_up)
        {
            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);

        }
        else if (gamepad1.dpad_down)
        {
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);
        }
        else
        {
            intakeLeft.setPower(0.0);
            intakeRight.setPower(0.0);
        }
//        telemetry.addData("intake", gamepad1.dpad_up);
//        telemetry.update();

    }
    public void runClaw()
    {

        double rightStickY = gamepad2.right_stick_y;
        Constants.ARM_MOTOR_TICKS = motorArm.getCurrentPosition();


        if(rightStickY > Constants.MINIMUM_JOYSTICK_PWR)
        {
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setPower(rightStickY * Constants.ARM_PWR_FACTOR);
        }
        else if(rightStickY < -Constants.MINIMUM_JOYSTICK_PWR)
        {
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setPower(-rightStickY * Constants.ARM_PWR_FACTOR);
        }
        else
        {
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArm.setTargetPosition(motorArm.getCurrentPosition());
        }


        telemetry.addData("ticks",Constants.ARM_MOTOR_TICKS);
        telemetry.update();


//        double deltaMotorPos = motorArm.getCurrentPosition() / Constants.ARM_MOTOR_TICKS;
//        // Power the servo such that it remains parallel to the ground.
//        servoJoint.setPosition(Constants.PARALLEL_SERVOJOINT_INIT + deltaMotorPos * 2);
//



    }

    void sendTelemetry()
    {

        telemetry.addData("left stick x:", gamepad1.left_stick_x);
        telemetry.addData("left stick y:", gamepad1.left_stick_y);
        telemetry.addData("right stick x:", gamepad1.right_stick_x);
        telemetry.addData("right stick y:", gamepad1.right_stick_y);
        telemetry.addData("right trigger:", gamepad2.right_trigger);
        telemetry.addData("left trigger:", gamepad2.left_trigger);
        telemetry.update();
    }
}
