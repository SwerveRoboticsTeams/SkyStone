package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract class MasterTeleOp extends Master
{

    boolean justSwitchedRunMode = true;

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
        telemetry.addData("intake", gamepad1.dpad_up);
        telemetry.update();

    }
    public void runClaw()
    {
        double armTicks;
        armTicks = motorArm.getCurrentPosition();

//        if (gamepad2.left_trigger > 0.4)
//        {
//            motorArm.setPower(gamepad2.left_trigger * 0.1);
//            telemetry.addData("arm encoder ticks", armTicks);
//            telemetry.update();
//
//        }else{
//
//        }
//        if (gamepad2.right_trigger > 0.4)
//        {
//            motorArm.setPower(-gamepad2.right_trigger * 0.1);
//            telemetry.addData("arm encoder ticks", armTicks);
//            telemetry.update();
//
//        }else{
//        }
        if(gamepad2.right_trigger > 0.1)
        {
            if(justSwitchedRunMode)
            {
                motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                justSwitchedRunMode = false;
            }
            motorArm.setPower(gamepad2.right_trigger * 0.25);
        }
        else if(gamepad2.left_trigger > 0.1)
        {
            if(justSwitchedRunMode)
            {
                motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                justSwitchedRunMode = false;
            }
            motorArm.setPower(-gamepad2.left_trigger * 0.25);
        }
        else if(!(gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1) && !justSwitchedRunMode)
        {
            justSwitchedRunMode = true;
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArm.setPower(1);
            motorArm.setTargetPosition(motorArm.getCurrentPosition());
        }

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
