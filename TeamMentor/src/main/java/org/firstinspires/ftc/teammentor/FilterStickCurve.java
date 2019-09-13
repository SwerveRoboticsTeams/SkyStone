package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by steve on 6/10/17.
 *
 * The purpose of this class is to provide various curves to alter the control stick range for better robot handling.
 */

public class FilterStickCurve {

    OpMode opMode; //allows us to use opMode.telemetry for debugging and seeing what the algorithm is doing

    public enum Curve
    {
        Square,  // n^2 * sign of n
        Cube     // n^3
    }

    Curve selectedCurve = Curve.Cube;

    public FilterStickCurve(OpMode opmode, Curve curve)
    {
        opMode = opmode;
        selectedCurve = curve;
    }

    public void setFilterCurve(Curve curve)
    {
        selectedCurve = curve;
    }

    public double getFilteredValue(final double controlInput)
    {
        //TODO replace with math
        double filteredControlValue = 0;

        switch (selectedCurve)
        {
            case Square:
                filteredControlValue = controlInput * controlInput * Math.signum(controlInput);
                break;
            case Cube:
                filteredControlValue = controlInput * controlInput * controlInput;
                break;
            default:
                break;

        }

        return filteredControlValue;
    }

    public void resetFilter()
    {
        //nothing to do
    }

    private String formatNumber(double d)
    {
        return String.format("%.8f", d);
    }

}
