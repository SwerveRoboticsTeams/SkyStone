package org.firstinspires.ftc.team417_2019;

import org.firstinspires.ftc.team417_2019.Resources.AvgFilter;
import org.firstinspires.ftc.team417_2019.Resources.Toggler;

abstract public class MasterTeleOp extends MasterOpMode
{
    double x = 0;
    double y = 0;
    double rotationalPower = 0;

    boolean isReverseMode = false;
    boolean isSlowMode = false;

    Toggler slowMode = new Toggler();
    Toggler reverseMode = new Toggler();


    AvgFilter filterJoyStickInput = new AvgFilter();


    void driveRobot()
    {
        if (slowMode.getToggle(gamepad1.right_bumper)) {
            isSlowMode = !isSlowMode;
        }
        if (reverseMode.getToggle(gamepad1.left_bumper)) {
            isReverseMode = !isReverseMode;
        }

        y = -gamepad1.right_stick_y; // Y axis is negative when up
        x = gamepad1.right_stick_x;
        rotationalPower = gamepad1.left_stick_x;

        if (isSlowMode) {
            y *= 0.2;
            x *= 0.2;
            rotationalPower *= 0.2;
        }
        else if (isReverseMode) {
            y *= -1;
            x *= -1;

        }


        // todo check and test to see if we need filtering
        /*
        filterJoyStickInput.appendInput(x, y, pivotPower);

        x = filterJoyStickInput.getFilteredX();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();
         */

        double drivePower = Math.hypot(x, y);
        double angle = Math.atan2(y,x);

        mecanumDrive(angle, drivePower, rotationalPower);
    }

    void foundationPullers()
    {

        if(puller.getToggle(gamepad1.a))
        {
            leftFoundationPuller.setPosition(0.65);
            rightFoundationPuller.setPosition(0.35);
        }
        else
        {
            leftFoundationPuller.setPosition(1);
            rightFoundationPuller.setPosition(0);
        }

    }

    void collector()
    {

    }

    void updateTelemetry()
    {
        telemetry.addData("legato: ", isSlowMode);
        telemetry.addData("reverse: ", isReverseMode);
        telemetry.update();
    }
}
