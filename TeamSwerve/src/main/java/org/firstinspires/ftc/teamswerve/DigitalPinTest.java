package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/*
    A simple opmode that shows how to use a digital input/output pin.
    This could be used to control a relay, an LED, etc.
 */


@TeleOp(name = "Digital Pin Test", group = "Tests")
@Disabled
public class DigitalPinTest extends LinearOpMode
{

    DigitalChannel myRelay;

    public void runOpMode() throws InterruptedException
    {

        //get the digital channel object
        myRelay = hardwareMap.digitalChannel.get("relay");

        //tell the hardware whether the pin should be an input or output.
        //Use inputs to read from devices, such as a switch.
        //Use outputs to control devices, such as LEDs or relays.
        myRelay.setMode(DigitalChannel.Mode.OUTPUT);

        //We're using an output because we're controlling a relay.
        //You control the output of the digital pin by setting its state.
        myRelay.setState(false);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                myRelay.setState(true);
                telemetry.log().add("relay turned on");
            }
            if(gamepad1.b)
            {
                myRelay.setState(false);
                telemetry.log().add("relay turned off");
            }

            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.update();
            idle();
        }

    }
}
