package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Program used to test a servo
 */
@TeleOp(name="CRServoTest", group = "Swerve")
//@Disabled
public class CRServoTest extends LinearOpMode {
    CRServo servo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and other important things
        servo = hardwareMap.crservo.get("servo");

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            servo.setPower(0.5);
            //pause(1000);
            //servo.setPower(1);
            //pause(1000);
            idle();
        }
    }

    //wait a number of milliseconds
    public void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while((System.nanoTime() - initialTime)/1000/1000 < t)
        {
            idle();
        }
    }



}
