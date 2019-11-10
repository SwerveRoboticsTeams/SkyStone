package org.firstinspires.ftc.team8923_2019;

/*
    This is implemented by any control filter classes.
*/

public interface Filter
{
    void roll(double newValue);

    double getFilteredValue();
}
