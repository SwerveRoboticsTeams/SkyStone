package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamswerve.FileWriter;

@TeleOp(name = "Dryw FileWriter Test", group = "Tests")
@Disabled
public class FileTest extends LinearOpMode
{
    FileWriter fileWriter;
    boolean lastButton = false;

    public void runOpMode() throws InterruptedException
    {

        waitForStart();

        fileWriter = new FileWriter("FileWriter Test " + System.currentTimeMillis() + ".txt");
        fileWriter.print("Hello file!");
        fileWriter.println("Hello file! Again!");
        fileWriter.println(Long.toString(System.currentTimeMillis()));
        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                //String message = Long.toString(System.currentTimeMillis());
                String message = System.currentTimeMillis() + " ";
                fileWriter.println(message);
                //fileWriter.println("hello");
                telemetry.log().add(message);
            }

            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.update();
            idle();
        }

        fileWriter.closeFile();
    }
}
