package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by steve on 6/10/17.
 *
 * The purpose of this class is to provide a PID filter.
 */

public class FilterPID {
    //constants for use in the PID algorithm.

    private double KP = 0; //weight to give to the Proportional term
    private double KI = 0; //weight to give to the Integral term
    private double KD = 0; //weight to give to the Derivative term

    //last observed value, for use with the D term
    private double lastObservedError = 0;

    //sum of all errors ever seen, for use with the I term.
    private double sumOfAllErrors = 0;

    private OpMode opMode; //allows us to use opMode.telemetry for debugging and seeing what the algorithm is doing


    public FilterPID(OpMode opmode, double P, double I, double D)
    {
        opMode = opmode;

        KP = P;
        KI = I;
        KD = D;
    }


    //calculate the results of the PID filter.
    //When used in turning, this value is typically applied to motor powers as follows:
    // double result = getFilteredValue(error);
    // motorLeftPower  -= result;
    // motorRightPower += result;
    public double getFilteredValue(double observedError)
    {

        //keep track of the sum of all errors ever seen, for the I term.
        sumOfAllErrors += lastObservedError;

        //calculate the difference between this error and the last error, for the D term.
        double differenceBetweenLastTwoErrors = observedError - lastObservedError;

        //update our lastObservedError with the new value, for the next time this filter is called.
        lastObservedError = observedError;

        //calculate the result of the PID filter
        double filteredValue = (KP * observedError) + (KI * sumOfAllErrors) + (KD * differenceBetweenLastTwoErrors);

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

        return filteredValue;
    }

    public void updateFilterConstants(double newKP, double newKI, double newKD)
    {
        KP = newKP;
        KI = newKI;
        KD = newKD;
    }

    public void resetFilter()
    {
        lastObservedError = 0;
        sumOfAllErrors = 0;
    }

    public double getP() {return KP;}
    public double getD() {return KD;}
    public double getI() {return KI;}

    private String formatNumber(double d)
    {
        return String.format("%.8f", d);
    }

}
