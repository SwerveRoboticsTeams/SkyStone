package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by steve on 6/10/17.
 *
 * The purpose of this class is to provide a filter that limits the
 * change in motor power to a certain rate per second
 * so the robot won't accelerate faster than its wheels can grip.
 */

public class FilterLimitAcceleration {

    private double maxAccelChangePerMillisecond = 1.0;
    private double maxDecelChangePerMillisecond = 1.0;

    //the last control value we allowed
    double lastControlValue = 0;

    //variables to keep track of the last time we were called.
    ElapsedTime timer;
    double millisecondsWhenLastCalled;

    //If we haven't been called in a really long time, here's a max value of elapsed time for use in our calculations
    //so the robot doesn't think it can accelerate far more than it should.
    //This value is slightly tunable, in that bigger values will allow the first iteration to accelerate/decelerate more.
    //If this value is too small, however, it will always be used instead of the actual elapsed time.
    final private int MAX_TYPICAL_LOOP_TIME_MS = 60;

    OpMode opMode; //allows us to use opMode.telemetry for debugging and seeing what the algorithm is doing


    public FilterLimitAcceleration(OpMode opmode, double maxAccelControlChangePerMillisecond, double maxDecelControlChangePerMillisecond)
    {
        opMode = opmode;
        maxAccelChangePerMillisecond = maxAccelControlChangePerMillisecond;
        maxDecelChangePerMillisecond = maxDecelControlChangePerMillisecond;
        timer = new ElapsedTime();
        millisecondsWhenLastCalled = 0.0;
    }

    public double getFilteredValue(final double controlInput)
    {
        final double millisecondsNow = timer.milliseconds();

        //determine how much time has passed since the last time we were called
        //if it's been a long time since this method was last called, reset everything
        double elapsedMilliseconds = millisecondsNow - millisecondsWhenLastCalled;
        if (elapsedMilliseconds > MAX_TYPICAL_LOOP_TIME_MS) {
            //opMode.telemetry.addData("over", elapsedMilliseconds);
            elapsedMilliseconds = MAX_TYPICAL_LOOP_TIME_MS;
            lastControlValue = 0;
        }

        //determine how much the control value has changed since last call
        final double changeInControlValue = controlInput - lastControlValue;

        double allowableChange = 0.0;
        //double amountToAdjustControlValue = 0.0;

        //is the magnitude of the control increasing?
        if (lastControlValue == controlInput)
        {
            //no change to control value since last call. We are neither accelerating nor decelerating.
            allowableChange = 0.0;
            //opMode.telemetry.addData("case", "zero");
        }
        else if ( ((Math.signum(controlInput) > 0) & (lastControlValue < controlInput)) ||
             ((Math.signum(controlInput) < 0) & (lastControlValue > controlInput))  )
        {
            //accelerating
            allowableChange = elapsedMilliseconds * maxAccelChangePerMillisecond;
            allowableChange = Math.min(Math.abs(allowableChange), Math.abs(changeInControlValue)) * Math.signum(changeInControlValue);
            //opMode.telemetry.addData("case", "accel");
        }
        else
        {
            //decelerating
            allowableChange = elapsedMilliseconds * maxDecelChangePerMillisecond;
            allowableChange = Math.min(Math.abs(allowableChange), Math.abs(changeInControlValue)) * Math.signum(changeInControlValue);
            //opMode.telemetry.addData("case", "decel");
        }

        final double filteredControlValue = lastControlValue + allowableChange;

        //store state variables for the next time we're called
        millisecondsWhenLastCalled = millisecondsNow;
        lastControlValue = filteredControlValue;

        /*
        opMode.telemetry.addData("elapsed: ", formatNumber(elapsedMilliseconds))
                .addData("control: ", formatNumber(controlInput))
                .addData("last: ", formatNumber(lastControlValue))
                .addData("change: ", formatNumber(changeInControlValue))
                .addData("allow: ", formatNumber(allowableChange))
                .addData("result: ", formatNumber(filteredControlValue))
                ;
        */
        //opMode.telemetry.update(); //let the main loop do the telemetry.update() call.

        return filteredControlValue;
    }

    public void resetFilter()
    {
        lastControlValue = 0;
        millisecondsWhenLastCalled = 0;
    }

    private String formatNumber(double d)
    {
        return String.format("%.8f", d);
    }

}
