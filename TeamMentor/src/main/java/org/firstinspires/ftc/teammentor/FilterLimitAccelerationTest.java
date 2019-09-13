package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an opmode that tests whether an encoder is working.
 */

@TeleOp(name="Test FilterLimitAccel", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class FilterLimitAccelerationTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        //Set up a filter to limit acceleration and deceleration.
        //I'm using very small values here so the results of the filter are easier to see
        FilterLimitAcceleration filter = new FilterLimitAcceleration(this, 0.00001, 0.0001);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double controlInput;
        double filteredValue;
        boolean filterIsOn = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.b) filter.resetFilter();

            if (gamepad1.a)
            {
                while (gamepad1.a)
                {
                    idle();
                }; //do nothing while the button is being held down

                filterIsOn = !filterIsOn;
            }

            if (filterIsOn)
            {

                controlInput = gamepad1.left_stick_y;
                filteredValue = filter.getFilteredValue(controlInput);

                telemetry.addData("state", "filter on");
                telemetry.addData("control", formatNumber(controlInput));
                telemetry.addData("filtered", formatNumber(filteredValue));
            }
            else
            {
                telemetry.addData("state", "filter off");
            }

            telemetry.update();
            idle();

        }
    }

    private String formatNumber(double d)
    {
        return String.format("%.8f", d);
    }
}
