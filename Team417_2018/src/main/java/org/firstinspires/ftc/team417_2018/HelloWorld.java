package org.firstinspires.ftc.team417_2018;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


//@TeleOp(name="TestServos", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class HelloWorld extends LinearOpMode
{

    CRServo vex1 = null; // port 0
    CRServo vex2 = null; // port 1

    public void runOpMode()
    {
        // initialize servos
        vex1 = hardwareMap.crservo.get("vex1");
        vex2 = hardwareMap.crservo.get("vex2");
        telemetry.addData("init", "done");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        vex1.setPower(0.0);
        vex2.setPower(0.0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
           if(gamepad1.a)
           {
               vex1.setPower(0.79);
               vex2.setPower(0.79);
           }
           else if (gamepad1.b)
           {
               vex1.setPower(-0.79);
               vex2.setPower(-0.79);
           }
           else
               {
               vex1.setPower(0);
               vex2.setPower(0);
           }
        }
    }
}