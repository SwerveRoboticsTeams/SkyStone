package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Program used to control a robot that draws what it sees through the robot phone camera.
 */
@TeleOp(name="Drawbot", group = "Swerve")
// @Disabled
public class SteveDrawbot extends LinearOpMode
{

    /*

    Our drawing bot has the following hardware construction:

    servo 1 (s1) is connected to a base and rotates an arm of length a ("side a").
    servo 2 (s2) is at the end of that arm, and rotates another arm of length b ("side b").
    * is the pen


                                  *
                                  |
                                  | side b
                      side a      |
          servo1 ------------------ servo2

     [hmmm, we need a way to lift and lower the pen...]

     Let alpha be the angle of servo1 relative to a baseline
     and let beta be the angle of servo2 relative to side a (the bar between servo1 and servo2).

     We can calculate the required angles using trig. (more about that below)

     We will use vuforia to "look" at a piece of white paper that has a drawing in black ink.
     We will obtain a bitmap from vuforia, scan the lines of the bitmap for non-white pixels,
     and have the drawbot ink those locations on a destination piece of paper.

     To "draw" the image, we need to convert the desired coordinates for the pen into
     the two angles (i.e., servo positions for servo1 and servo2).

     There is a handy article here that explains the math.
     https://math.stackexchange.com/questions/1423467/calculating-angles-neccessary-to-reach-a-position-on-a-2d-plane-for-two-robot-ar


     */


    Servo servo1, servo2;

    //the hardware may not allow the servos to be perfectly aligned with where they should be,
    //... so let's add some adjustments and limits in case we need them
    final private double SERVO_1_OFFSET = 0.0;
    final private double SERVO_1_MIN = 0;
    final private double SERVO_1_MAX = 1.0;
    final private double SERVO_2_OFFSET = 0.1;
    final private double SERVO_2_MIN = 0.1;
    final private double SERVO_2_MAX = 1.0;


    //with these lengths 0,0 and 9,10 (or 10,9) are not reachable, but most coords in between are.
    //however, some coords will result in the hardware hitting itself, which I haven't protected against yet.
    Double len_a = 7.0; //length of bar from servo1 to servo2
    Double len_b = 6.0;  //length of bar from servo2 to pen
    double len_c = 0.0;  //length from servo1 to pen. This will be calculated as needed for each point of the drawing.

    //     Let alpha be the angle of servo servo1 relative to a baseline
    //     and let beta be the angle of servo servo2 relative to side a (the bar between servo1 and servo2).
    public double angleAlpha;
    public double angleBeta;

    double DEFAULT_ANGLE_ALPHA = 0.0;
    double DEFAULT_ANGLE_BETA = 0.0;

    // some variables to turn the pen on and off
    final boolean PEN_DRAW = true;
    final boolean PEN_NOT_DRAW = false;
    boolean penState = PEN_NOT_DRAW;

    final private boolean RUN_WITH_HARDWARE = true;


    @Override public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        drawSquare();


        // Main loop
        while(opModeIsActive())
        {
            //nothing yet, this is just a stub


            telemetry.update();
            idle();
        }
    }

    private void moveWithWait(double x, double y)
    {
        moveTo(x, y);
        waitMS(5000);
    }


    private void testPatternOne() {
        moveWithWait(3, 3);
        moveWithWait(4, 4);
        moveWithWait(5, 5);
        moveWithWait(6, 6);
    }

    private void drawSquare()
    {
        moveWithWait(4, 4);
        moveWithWait(4, 7);
        moveWithWait(7, 7);
        moveWithWait(7, 4);
        moveWithWait(4, 4);
    }

    private void testLimits()
    {
        //This method is best run with RUN_WITH_HARDWARE = false.
        //We're counting on error handling in other methods to tell us what coords are out of range
        //for the given magnitudes of len_a and len_b
        for (int x = 0; x<11; x++)
            for (int y=0; y<11; y++)
                moveTo(x,y);
    }


    private void initializeServos()
    {
        if (RUN_WITH_HARDWARE) {
            servo1 = hardwareMap.servo.get("servo1");
            servo2 = hardwareMap.servo.get("servo2");
        }

        angleAlpha = DEFAULT_ANGLE_ALPHA;
        angleBeta = DEFAULT_ANGLE_BETA;

        updateServoPositions();
    }


    private void initializeRobot()
    {
        initializeServos();

        // Set up telemetry data
        telemetry.log().setCapacity(400);
    }


    private boolean validServo1Position(double pos)
    {
        if ( (pos >= SERVO_1_MIN) && (pos <=SERVO_1_MAX)) return true;
        else return false;
    }

    private boolean validServo2Position(double pos)
    {
        if ( (pos >= SERVO_2_MIN) && (pos <=SERVO_2_MAX)) return true;
        else return false;
    }

    private void updateServoPositions()
    {
        //adjust for hardware orientation
        double s1AdjustedPosition = (1 - angleAlpha) + SERVO_1_OFFSET;
        double s2AdjustedPosition = angleBeta + SERVO_2_OFFSET;

        if (validServo1Position(s1AdjustedPosition)  && validServo2Position(s2AdjustedPosition) )
        {
            if (RUN_WITH_HARDWARE)
            {
                servo1.setPosition(s1AdjustedPosition);
                servo2.setPosition(s2AdjustedPosition);
            }
            telemetry.log().add("alpha " + formatNumber(angleAlpha) + ", beta " + formatNumber(angleBeta));
        }
        else
        {
            telemetry.log().add("Out of Range: " + formatNumber(s1AdjustedPosition) + ", " + formatNumber(s2AdjustedPosition));
            telemetry.update();
        }

    }

    //put the pen in a state in which it does not draw lines
    private void setPenState(boolean desiredPenState)
    {
        penState = desiredPenState;
    }

    private void calculateServoAnglesForPoint(double x, double y)
    {
        //cribbing heavily from:
        // https://math.stackexchange.com/questions/1423467/calculating-angles-neccessary-to-reach-a-position-on-a-2d-plane-for-two-robot-ar

        //c is the distance from the origin to the target location (x,y).
        //The formula for the distance between two points is  Math.sqrt( (x2 - x1)^2 + (y2 - y1)^2 )
        //Luckily, the origin is conveniently (0,0), which simplifies the formula for us.
        double c = Math.sqrt( (x * x) + (y * y) );

        double betaInRadians = Math.acos(
                ((len_a * len_a) + (len_b * len_b) - (c * c)) /
                                (2 * len_a * len_b)
        );


        double g = Math.atan2(x, y);
        double t = Math.acos(
                ((len_a * len_a) + (c * c) - (len_b * len_b))  /
                               (2 * len_a * c)
        );

        double alphaInRadians = g + t;

        angleAlpha = convertRadiansToServoRange(alphaInRadians);
        angleBeta = convertRadiansToServoRange(betaInRadians);

    }

    private double convertRadiansToServoRange(double rads)
    {
        //convert radians to degrees
        double degrees = rads * (180.0 / Math.PI);

        //these servos turn 180 degrees; map degrees to a value from 0..1
        return degrees / 180.00;
    }

    private void moveTo(double x, double y)
    {
        setPenState(PEN_NOT_DRAW);

        telemetry.log().add("x " + x + ", y " + y);

        calculateServoAnglesForPoint(x, y);

        updateServoPositions();
    }


    //note that this may draw a curving line as the servos move to their new positions
    private void drawTo(double x, double y)
    {
        setPenState(PEN_DRAW);

        calculateServoAnglesForPoint(x, y);

        updateServoPositions();
    }

    void waitMS(double millisecondsToWait)
    {
        ElapsedTime timer = new ElapsedTime();

        while(opModeIsActive() && (timer.milliseconds() < millisecondsToWait))
        {
            telemetry.addData("Countdown (ms):", millisecondsToWait - timer.milliseconds());
            telemetry.update();
            idle();
        }
    }


    private String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
    private String formatNumberEightDigits(double d)
    {
        return String.format("%.8f", d);
    }
}
