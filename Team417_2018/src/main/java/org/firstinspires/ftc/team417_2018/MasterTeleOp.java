package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

abstract public class MasterTeleOp extends MasterOpMode
{
    double x = 0;
    double y = 0;
    double pivotPower = 0;

    double curRevPos = INIT_REV_POS; // starts in down position
    double autoRevPos = 0.0;
    double autoDouble = 0.0;

    final double ADAGIO_POWER = 0.3;

    boolean isReverseMode = false;
    boolean isLegatoMode = false;

    boolean isRightBumperPushed = false;
    boolean isSuckingIn = false;


    boolean isDpadLeftPushed = false;
    boolean isCollectingPos = false;

    boolean isCollectorDown = false; // gamepad joystick up

    boolean isMarkerDown = true;
    boolean isExtending = false;
    boolean isYButtonPressed = true;

    final double Krev = -1/1210.0;
    int targetCorePos = 0;

    AvgFilter filterJoyStickInput = new AvgFilter();


    void mecanumDrive()
    {
        // hold right bumper for adagio legato mode
        if (gamepad1.right_trigger>0) isLegatoMode = true;
        else isLegatoMode = false;
        // hold left bumper for reverse mode
        if (gamepad1.left_trigger>0) isReverseMode = true;
        else isReverseMode = false;


        if (isLegatoMode) // Legato Mode
        {
            y = -Range.clip(-gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            x = -Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
            if (gamepad1.dpad_left) x = -0.2;
            if (gamepad1.dpad_right) x = 0.2;
            if (gamepad1.dpad_down) y = -0.2;
            if (gamepad1.dpad_up) y = 0.2;
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.2, 0.2);

            if (isReverseMode) // if both legato and reverse mode
            {
                y = Range.clip(-gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
                x = Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
                if (gamepad1.dpad_left) x = -0.3;
                if (gamepad1.dpad_right) x = 0.3;
                if (gamepad1.dpad_down) y = -0.3;
                if (gamepad1.dpad_up) y = 0.3;
                pivotPower = Range.clip(gamepad1.left_stick_x, -0.3, 0.3);
            }
        }
        else if (isReverseMode)
        {
            y = Range.clip(-gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            x = Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
            if (gamepad1.dpad_left) x = -0.75;
            if (gamepad1.dpad_right) x = 0.75;
            if (gamepad1.dpad_down) y = -0.75;
            if (gamepad1.dpad_up) y = 0.75;
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.3, 0.3);
        }
        else // Staccato Mode
        {
            y = gamepad1.right_stick_y; // Y axis is negative when up
            x = -gamepad1.right_stick_x;
            if (gamepad1.dpad_left) x = -0.75;
            if (gamepad1.dpad_right) x = 0.75;
            if (gamepad1.dpad_down) y = -0.75;
            if (gamepad1.dpad_up) y = 0.75;
            if (gamepad1.right_stick_y != 0) y = gamepad1.right_stick_y;
            //pivotPower = Range.clip(gamepad1.left_stick_x, -0.9, 0.9);
            pivotPower = (gamepad1.left_stick_x) * 0.95;
        }

        filterJoyStickInput.appendInput(x, y, pivotPower);

        x = filterJoyStickInput.getFilteredX();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();

        powerFL = -x - y + pivotPower;
        powerFR = x - y - pivotPower;
        powerBL = x - y + pivotPower;
        powerBR = -x - y - pivotPower;

        motorFL.setPower(powerFL);
        motorBL.setPower(Range.clip(powerBL,-0.6,0.6));
        motorFR.setPower(powerFR);
        motorBR.setPower(powerBR);
    }

    void linearSlides()
    {
        // control the extending with G2 right (extend) and left (retract) trigger
        //if (gamepad2.right_trigger != 0 && targetCorePos < MAX_CORE_POS) core2.setPower(0.7); // extend
        //if (gamepad2.left_trigger != 0 && targetCorePos > MIN_CORE_POS) core2.setPower(-0.8); // retract

        // control the extending with G2 right (extend) and left (retract) trigger
        if (gamepad2.right_trigger != 0)
        {
            core2.setPower(0.99);
        }
        else if (gamepad2.left_trigger != 0)
        {
            isExtending = false;
            core2.setPower(-0.99);
        }
        else if (gamepad2.left_trigger==0 && gamepad2.right_trigger==0 && !isExtending)
        {
            core2.setPower(0.0);
        }

        if (arm1.getCurrentPosition() > -790) isExtending = false;

        // Press button y to toggle up and down
        if (gamepad2.y && !isExtending)
        {
            isYButtonPressed = true;
            isExtending = !isExtending;
        }
        isYButtonPressed = gamepad2.y;
        if (isExtending && arm1.getCurrentPosition() < -790) core2.setPower(0.1); // extends with toggle

    }

    void collector()
    {
        // control arm motors with G2 right stick
        if (gamepad2.right_stick_y != 0)
        {
            arm1.setPower(Range.clip(gamepad2.right_stick_y, -0.5, 0.5));
            arm2.setPower(Range.clip(-gamepad2.right_stick_y, -0.5, 0.5));
        }
        else
        {
            arm1.setPower(0.0);
            arm2.setPower(0.0);
        }

// Set Automatic Rev servo position
        autoDouble = (double) (arm1.getCurrentPosition());
        autoDouble /=-1465.5;
        autoRevPos =  autoDouble + 0.1; // high pos = -1058, low = 0
        rev1.setPosition(autoRevPos);

// control hanger with G2 left and right bumpers
        if (gamepad2.dpad_up)
        {
            hanger.setPower(0.99); // extend the hanger
        }
        else if (gamepad2.dpad_down)
        {
            hanger.setPower(-0.99); // retract hanger
        }
        else
        {
            hanger.setPower(0.0);
        }

        if (gamepad2.left_bumper) vex1.setPower(0.79); // spit
        else if (gamepad2.right_bumper) vex1.setPower(-0.79); // suck
        else vex1.setPower(0.0);

        /*
        if (gamepad2.left_bumper)
        {
            isSuckingIn = false; // cancel sucking in
            vex1.setPower(0.79); // if you press the left bumper release minerals
        }
        if (!gamepad2.left_bumper && !isSuckingIn) vex1.setPower(0.0); // if you are not pressing the left bumper do not set power to the vex motor

        if(isSuckingIn) vex1.setPower(-0.79); // if sucking in is true then set negative power so the servo spins opposite way

        if (gamepad2.right_bumper && !isRightBumperPushed) // if the right bumper is pressed && boolean for isRightBumperPushed true( that means it is currently false)
        {
            isRightBumperPushed = true; // switch the value of right bumper pushed to true
            isSuckingIn = !isSuckingIn; // and switch sucking in's boolean to sucking out or in depending on what it is
        }
        isRightBumperPushed = gamepad2.right_bumper; // update the current state of isRightBumperPushed otherwise it will always stay the same
        */
    }

    void revServo()
    {
        // control rev servo with G2 left stick
        if (-gamepad2.left_stick_y > 0.1 && curRevPos > 0.0) // if the joystick is UP
        {
            isCollectingPos = false;
            curRevPos = curRevPos - REV_INCREMENT; // move the collector up
        }
        else if (-gamepad2.left_stick_y < -0.1 && curRevPos < 0.9) // if the joystick is DOWN
        {
            isCollectingPos = false;
            curRevPos = curRevPos + REV_INCREMENT; // move the collector down
        }
        rev1.setPosition(curRevPos); // set the wrist REV servo position

    }

    void autoRev()
    {
        double autoRevPosDoub = (double) arm1.getCurrentPosition();
        autoRevPos = (  autoRevPosDoub - 60.0) / Krev;
        rev1.setPosition(autoRevPos);
    }

    void marker()
    {
        // Press button y to toggle up and down
        if(isMarkerDown) marker.setPosition(MARKER_LOW);
        else marker.setPosition(MARKER_HIGH);

        if (gamepad2.y && !isYButtonPressed)
        {
            isYButtonPressed = true;
            isMarkerDown = !isMarkerDown;
        }
        isYButtonPressed = gamepad2.y;
    }

    void updateTelemetry()
    {
        //telemetry.addData("legato: ", isLegatoMode);
        telemetry.addData("reverse: ", isReverseMode);
        telemetry.addData("hanger:", hanger.getCurrentPosition());
        telemetry.addData("autoRevPos:", autoRevPos);
        //telemetry.addData("motorMode", core2.getMode());
        telemetry.addData("core2:", core2.getCurrentPosition());
        telemetry.addData("arm1:", arm1.getCurrentPosition());
        telemetry.addData("rev1", rev1.getPosition());
        telemetry.update();
    }
}
