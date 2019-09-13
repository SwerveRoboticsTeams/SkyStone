package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous(name = "Photo Circuit Test",group = "Tests")
//@Disabled
public class PhotoCircuit extends LinearOpMode
{
    AnalogInput photoRelay;
    double voltage;

    public void runOpMode() throws InterruptedException
    {

        // Get the analog channel object
        photoRelay = hardwareMap.analogInput.get("photoRelay");

        waitForStart();


        while (opModeIsActive())
        {
            voltage = photoRelay.getVoltage();
            telemetry.addData("voltage: ", voltage);
            telemetry.update();
            idle();
        }
    }
}
