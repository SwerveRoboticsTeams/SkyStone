package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Random;

/**
 * This file contains an opmode that tests the performance impact of using various math calls.
 *
 * What do the digits in a time mean?
 *
 *     s        100ms    10ms      ms      1/10 ms
 *     0    .    0        0        0        0
 *
 *
 *    Calculation results on my ZTE speed:
 *
 *    An empty for loop running for  1,000,000 iterations takes ~ 0.0051
 *    An empty for loop running for 10,000,000 iterations takes ~ 0.051
 *
 *    Adding two integers 1,000,000 times takes ~0.0051
 *
 *    Adding two doubles  1,000,000 times takes ~0.0053
 *    ...however, there seemed to be more variability in this number, it was occasionally 0.006 or even 0.009
 *
 *    At this point I'll note that the numbers do generally have a little variability in them,
 *    and I'm reporting the value I see the most solidly in the telemetry output.
 *    Yes, I could calculate max, average, etc. and that would be more reliable,
 *    but since my goal is to get a general sense of how much calculations cost us,
 *    and we're doing 1,000,000 calculations per measurement,
 *    I think this is close enough for my purposes.
 *    Feel free to repeat these experiments with more rigor.
 *
 *    Multiplying two integers 1,000,000 times takes ~0.0052
 *    This number had even more variability, occasionally going as high as ~0.0171,
 *    but generally being around 0.0052
 *
 *    Multiplying two doubles 1,000,000 times takes ~0.0052
 *    This number had around the same variability as the previous test, occasionally going as high as ~0.0110
 *
 *    I'm skipping an integer division test because that's not a useful operation to do in a real robot
 *
 *    Dividing two doubles 1,000,000 times takes ~0.0166
 *
 *    Math.pow(d,e) 1,000,000 times takes ~0.50 seconds. This is an expensive call! But still cheap if you're doing one.
 *
 *    So... what slows our programs down? A likely culprit is creating/destroying objects every loop.
 *    You can test that hypothesis by creating/destroying your objects in the loop below and seeing how
 *    long that takes.
 *
 *    And, of course, calling methods takes time, too, since the method needs to be put on the stack,
 *    its instance variables need to be created, etc. That's another good area to test.
 *
 *
 */

@TeleOp(name="TestMath", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class TestMath extends LinearOpMode {

    @Override
    public void runOpMode() {

        Random rand = new Random();

        int x = rand.nextInt();
        int y = rand.nextInt();
        int z = 0;
        double d = rand.nextDouble();
        double e = rand.nextDouble();
        double f = 0.0;


        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime loopTime = new ElapsedTime();

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            loopTime.reset();

            for (int i = 0; i < 1000000; i++)
            {

                /*
                      Uncomment the operation you want to measure
                 */

                //z = x + y;
                //f = d + e;

                //z = x * y;
                //f = d * e;

                //z = x / y;  //why bother testing this? Integer division should be avoided.
                //f = d / e;

                //Math.pow(d, e);
            }


            telemetry.addData("Status", "Loop Time: " + loopTime.toString());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("vars", "x,y,z = " + x + "," + y +"," + z);
            telemetry.addData("vars", "d,e,f = " + d + "," + e +"," + f);

            telemetry.update();
            idle();

        }
    }
}
