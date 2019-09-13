package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;


/**
 * Program used to control Drive-A-Bots.
 * This can be a good reference for drive controls.
 */
@TeleOp(name="FileWriterTest", group = "Swerve")
@Disabled
public class FileWriterTest extends LinearOpMode
{

    @Override public void runOpMode() throws InterruptedException
    {
        // Wait until start button has been pressed
        waitForStart();

        //Open a file for writing our data into.
        //The file will appear on the robot phone in the folder storage/legacy/emulated
        FileWriter myFile = new FileWriter("testfile.txt");

        //write some data to a file
        for (int i=0; i<5; i++)
        {
            myFile.println("line " + i);
        }

        //close the file when you're done. (Not strictly necessary, but nice to do.)
        myFile.closeFile();

        // Main loop
        while(opModeIsActive())
        {
            telemetry.addData("done", "done");
            telemetry.update();
            idle();
        }
    }


}
