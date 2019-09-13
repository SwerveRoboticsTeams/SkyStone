package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract class MasterTeleOp extends Master
{
    boolean dankState = true;
    boolean JJUp = false;
    boolean JJ2Up = false;
    int flipSuccState = 0;
    boolean flipped = false;
    boolean justSwitchedFlipModes = true;

    void driveMecanumTeleOp()
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

    void dankUnderglow(double power)
    {
        motorDankUnderglow.setPower(power);
    }

    void fastFlex()
    {
        if (gamepad1.x)
        {
            dankUnderglow(1.0);
        }
        else
        {
            dankUnderglow(-1.0);
        }
        idle();
    }

    void runLift()
    {
        if(gamepad1.right_trigger > 0.35)
        {
            motorLift.setPower(1.0);
        }
        else if(gamepad1.left_trigger > 0.35)
        {
            motorLift.setPower(-1.0);
        }
        else
        {
            motorLift.setPower(0.0);
        }
    }
    void runFlipNSuccNPush()
    {
        // RUN FLIP
        //Manual control:
        if(gamepad2.right_trigger > 0.1)
        {
            if(justSwitchedFlipModes)
            {
                motorFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                justSwitchedFlipModes = false;
            }
            motorFlip.setPower(gamepad2.right_trigger * 0.25);
        }
        else if(gamepad2.left_trigger > 0.1)
        {
            if(justSwitchedFlipModes)
            {
                motorFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                justSwitchedFlipModes = false;
            }
            motorFlip.setPower(-gamepad2.left_trigger * 0.25);
        }
        else if(!(gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1) && !justSwitchedFlipModes)
        {
            justSwitchedFlipModes = true;
            motorFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFlip.setTargetPosition(motorFlip.getCurrentPosition());
        }
        //automatic half rotation
        else if(gamepad2.right_bumper && flipSuccState == 0 && !flipped)
        {
            motorFlip.setTargetPosition(motorFlip.getCurrentPosition() - 1120);
            //motorFlip2.setTargetPosition(motorFlip2.getCurrentPosition() - 1120);
            flipSuccState = 1;
        }
        else if(gamepad2.left_bumper && flipSuccState == 0 && flipped)
        {
            motorFlip.setTargetPosition(motorFlip.getCurrentPosition() + 1120);
            //motorFlip2.setTargetPosition(motorFlip2.getCurrentPosition() + 1120);
            flipSuccState = 1;
        }
        else if(flipSuccState == 0)
        {
            motorFlip.setPower(Math.min((motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) * (1.0 / 150), 0.5));
            //motorFlip2.setPower(Math.min((motorFlip2.getTargetPosition() - motorFlip2.getCurrentPosition()) * (1.0 / 50), 0.1));
        }

        if(flipSuccState == 1)
        {
            motorFlip.setPower(Math.signum(motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) *
                    Math.max(Math.min(Math.abs(motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) * (1.0 / 750), 0.3), 0.1));
            //motorFlip2.setPower(Math.signum(motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) *
            //Math.max(Math.min(Math.abs(motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) * (1.0 / 750), 0.5), 0.2));
            if (Math.abs(motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) < 20)
            {
                motorFlip.setTargetPosition(motorFlip.getCurrentPosition());
                //motorFlip2.setTargetPosition(motorFlip2.getCurrentPosition());
                flipSuccState = 0;
                flipped = !flipped;
            }
        }

        //RUN SUCC
        // inverted because y is inverted on the stick (up is negative)
        if(Math.abs(gamepad2.right_stick_y) > 0)
        {
            // multiplied by 0.85 to restrict the power to a max of 0.85 b/c Vex Motors are weird (possibly PWM funkiness)
            motorSucc1.setPower(gamepad2.right_stick_y * 0.85);
            motorSucc2.setPower(-gamepad2.right_stick_y * 0.85); // inverse of above b/c Vex Motor is rotated 180 degrees
        }
        else if(Math.abs(gamepad2.left_stick_y) > 0)
        {
            motorSucc1.setPower(gamepad2.left_stick_y * 0.35);
            motorSucc2.setPower(-gamepad2.left_stick_y * 0.35); // inverse of above b/c Vex Motor is rotated 180 degrees
        }
        else
        {
            motorSucc1.setPower(0.0);
            motorSucc2.setPower(0.0);
        }
    }

    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    void sendTelemetry()
    {
        telemetry.addData("arm ticks", motorFlip.getCurrentPosition());
        telemetry.addData("lift ticks", motorLift.getCurrentPosition());
        telemetry.addData("left stick x:", gamepad1.left_stick_x);
        telemetry.addData("left stick y:", gamepad1.left_stick_y);
        telemetry.addData("right stick x:", gamepad1.right_stick_x);
        telemetry.addData("right stick y:", gamepad1.right_stick_y);
        telemetry.addData("right trigger:", gamepad2.right_trigger);
        telemetry.addData("left trigger:", gamepad2.left_trigger);
        telemetry.addData("flip encoder:", motorFlip.getCurrentPosition());
        telemetry.update();
    }
}
