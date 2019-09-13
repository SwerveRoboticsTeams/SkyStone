package org.firstinspires.ftc.team417_2018;

public class AvgFilter
{
    private double avgX = 0;
    private double avgY = 0;
    private double avgP = 0;
    private double avgLY = 0;
    private double avgRY = 0;
    int numSamples = 5; // samples to average by

    double datX[] = new double[numSamples];
    double datY[] = new double[numSamples];
    double datP[] = new double[numSamples];
    double datLY[] = new double[numSamples];
    double datRY[] = new double[numSamples];


    public AvgFilter()
    {
        int i;
        for (i = 0; i < numSamples; i++)
        {
            datX[i] = 0.0;
            datY[i] = 0.0;
            datP[i] = 0.0;
        }
    }

    // append input
    public void appendInput(double x, double y, double p) // x, y, and p are the joystick inputs
    {
        int i;
        // shift new data into array
        for (i = 0; i < numSamples - 1; i++)
        {
            datX[i] = datX[i + 1];
            datY[i] = datY[i + 1];
            datP[i] = datP[i + 1];
        }
        // new value goes to last location
        datX[numSamples - 1] = x;
        datY[numSamples - 1] = y;
        datP[numSamples - 1] = p;

        // compute the average
        double sumX = 0.0;
        double sumY = 0.0;
        double sumP = 0.0;
        for (i = 0; i < numSamples; i++)
        {
            sumX += datX[i];
            sumY += datY[i];
            sumP += datP[i];
        }
        avgX = sumX / (double) numSamples;
        avgY = sumY / (double) numSamples;
        avgP = sumP / (double) numSamples;
    }

    // append input
    public void appendInputY (double ly, double ry) // y is the joystick input
    {
        int i;
        // shift new data into array
        for (i = 0; i < numSamples - 1; i++)
        {
            datLY[i] = datLY[i + 1];
            datRY[i] = datRY[i + 1];
        }
        // new value goes to last location
        datLY[numSamples - 1] = ly;
        datRY[numSamples - 1] = ry;

        // compute the average
        double sumLY = 0.0;
        double sumRY = 0.0;
        for (i = 0; i < numSamples; i++)
        {
            sumLY += datLY[i];
            sumRY += datRY[i];
        }
        avgLY = sumLY / (double) numSamples;
        avgRY = sumRY / (double) numSamples;
    }

    // get filtered x
    public double getFilteredX()
    {
        return avgX;
    }

    public double getFilteredY()
    {
        return avgY;
    }

    public double getFilteredP()
    {
        return avgP;
    }

    public double getFilteredLY()
    {
        return avgLY;
    }
    public double getFilteredRY()
    {
        return avgRY;
    }
}