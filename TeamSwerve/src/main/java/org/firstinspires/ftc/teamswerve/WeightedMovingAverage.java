package org.firstinspires.ftc.teamswerve;

import java.util.ArrayList;
import java.util.List;

/*
The purpose of this class is to filter noisy data.
 */
public class WeightedMovingAverage
{
    // Let's define the 0th element as the oldest.
    private ArrayList<Double> values; // declare list for data values
    private double[] weights; // declare array for all weights
    private int samples = 0; // declare variable for number of samples

    public WeightedMovingAverage(double[] weightArray)
    {
        samples = weightArray.length; // number of samples is the length of the weight array

        if (weightArray.length < 2) throw new IllegalArgumentException("Array requires at least 2 elements");

        //The elements of the weightArray must sum to 1.0.  (Since double arithmetic is not exact, we'll allow a slop factor.)
        //Confirm that the input weights sum to roughly 1.0 and throw an exception if not.
        double sumOfWeights = 0; // declare variable for sum of the weights

        for (int i = 0; i < samples; i++)
        {
            sumOfWeights += weightArray[i]; // add each weight to the sum of weights
        }
        // if the sum of weights are more than 1.001 or less than 0.999,
        if ((sumOfWeights > 1.001) || (sumOfWeights < 0.999))
        {
            // throw an exception telling the user the weights don't add up to one with a comment
            throw new IllegalArgumentException("Array elements must sum to 1.0");
        }

        values = new ArrayList<Double>(samples); // set list "values" to have the same index as the weight array
        weights = new double[samples]; // set weights to to have the same index as the "values" list

        // Reminder: the 0th element is the oldest one.
        for (int i = 0; i < samples; i++)
        {
            weights[i] = weightArray[i]; // set the weight array in this class to the weight array passed in as a parameter
        }
    }

    /*
    This method adds a new value to the value values list after getting rid of the 0th value, or the
    oldest value.  It only removes the last value if the list is full.
     */
    public void addNewValue(double newValue)
    {
        //get rid of oldest value
        //only remove the last element if we have already filled the values list
        //(the values list starts empty when this class is created)
        if (values.size() == samples)
        {
            removeLastElement(); // call the method below this one to remove the last element
        }
        values.add(newValue); // add new value to the end of the list
    }

    /*
    This method removes the last, or 0th, element from the values list.
     */
    private void removeLastElement()
    {
        if (values.size() > 0) // check if the values size is greater than 0
        {
            values.remove(0); // remove the 0th element
        }
    }

    /*
    This method applies the set weight to each sample inside of the filter.  The weight with the
    most impact on a sample is the weight closest to the newest value.
     */
    private double applyWeight(int index)
    {
        return values.get(index) * weights[index]; // multiply the values by the weight at each index
    }

    /*
    This method returns the running total after applying the weighted filter calculations.
     */
    public double getRunningTotal()
    {
        double runningTotal = 0.0; // declare a variable to keep track of the running total
        for (int i = 0; i < values.size(); i++) // for all of the values,
        {
            runningTotal += applyWeight(i); // add up all of the weighted values to the running total
        }
        return runningTotal; // return the running total
    }
}
