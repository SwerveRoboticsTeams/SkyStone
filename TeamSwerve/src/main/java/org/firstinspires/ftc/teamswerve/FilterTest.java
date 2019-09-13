package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
The purpose of this class is to test the WeightedMovingAverage class without any hardware except for
RC and DS phones.  The point is to simply display the running total of test data passed in to check
if the WeightedMovingAverage class is working properly.  Please use this class as a test when a
change is made.
 */
//@TeleOp(name="FilterTest", group = "Swerve") // The test shows up as "FilterTest" on the DS phone.
public class FilterTest extends LinearOpMode // This class extends LinearOpMode.
{
    @Override // tells the compiler we're overriding the base class method, because this is a subclass
    public void runOpMode() throws InterruptedException
    {
        waitForStart(); // wait until the start button is pushed

        double[] weights1 = new double[20]; // declare a weights array with an index of 20
        // set the 20 weights' values
        for (int i = 0; i < 20; i++)
        {
            weights1[i] = 1.0/10.0; // so you don't have to divide inside of the getRunningTotal method
        }
        // create an instance of the weighted moving average and pass in the weights that were set above
        WeightedMovingAverage filter1 = new WeightedMovingAverage(weights1);
        filter1.addNewValue(1.0); // add a value of one to the values list
        telemetry.addData("Filter1", filter1.getRunningTotal()); // display the running total on the DS screen
        telemetry.update(); // update the screen
        sleep(5000); // wait 5 seconds
    }
}