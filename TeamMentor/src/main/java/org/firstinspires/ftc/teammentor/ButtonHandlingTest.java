package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Test code for button processing
 */
@TeleOp(name="ButtonTest", group = "Swerve")
public class ButtonHandlingTest extends LinearOpMode
{
    enum Alliance
    {
        RED,
        BLUE;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {


        waitForStart();

        //chooseOptions();

        while (opModeIsActive())
        {
            //chooseOptions();
            chooseOptionsNoLoop();
            idle();
        }

    }

    Alliance alliance = Alliance.RED;
    int delayTime;

    //extra statements to help debug
    int loopCounter2 = 0;
    int buttonPresses2 = 0;


    void chooseOptionsNoLoop()
    {
        boolean setupFinished = false;

        //while (!setupFinished)
        {
            loopCounter2++;
            if (gamepad1.x)
            {
                alliance = Alliance.BLUE;
                buttonPresses2++;
            }
            else if (gamepad1.b)
            {
                alliance = Alliance.RED;
                buttonPresses2++;
            }

            if(gamepad1.dpad_up)
            {
                delayTime++;
                buttonPresses2++;
            }
            else if (gamepad1.dpad_down && delayTime > 0)
            {
                delayTime--;
                buttonPresses2++;
            }

            if(gamepad1.start)
                setupFinished = true;

            telemetry.addData("Alliance (Blue/Red): (X/B)", alliance.name());
            telemetry.addData("Delay Time (+/-): (Dpad Up/Dpad Down)", delayTime);
            telemetry.addData("counter", loopCounter2);
            telemetry.addData("buttonPresses", buttonPresses2);
            telemetry.update();
            idle();

            //spin until button has been released
            while (!buttonsAreReleased(gamepad1))
            {
                idle();
            }
        }
    }

    void chooseOptions()
    {
        boolean setupFinished = false;

        //extra statements to help debug
        int loopCounter = 0;
        int buttonPresses = 0;

        while (!setupFinished)
        {
            loopCounter++;
            if (gamepad1.x)
            {
                alliance = Alliance.BLUE;
                buttonPresses++;
            }
            else if (gamepad1.b)
            {
                alliance = Alliance.RED;
                buttonPresses++;
            }

            if(gamepad1.dpad_up)
            {
                delayTime++;
                buttonPresses++;
            }
            else if (gamepad1.dpad_down && delayTime > 0)
            {
                delayTime--;
                buttonPresses++;
            }

            if(gamepad1.start)
                setupFinished = true;

            telemetry.addData("Alliance (Blue/Red): (X/B)", alliance.name());
            telemetry.addData("Delay Time (+/-): (Dpad Up/Dpad Down)", delayTime);
            telemetry.addData("counter", loopCounter);
            telemetry.addData("buttonPresses", buttonPresses);
            telemetry.update();
            idle();

            //spin until button has been released
            while (!buttonsAreReleased(gamepad1))
            {
                idle();
            }
        }
    }

    boolean buttonsAreReleased(Gamepad pad)
    {
        return !(pad.a || pad.b || pad.x || pad.y || pad.left_bumper || pad.right_bumper
                || pad.dpad_up || pad.dpad_down || pad.dpad_left || pad.dpad_right
                || pad.left_stick_button || pad.right_stick_button
                || pad.start || pad.back || pad.guide || pad.left_trigger > 0.35
                || pad.right_trigger > 0.35);
    }
}
