package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Program used to control a simple robot arm.
 *
 * The arm consists of 3 servos:
 *      a "base" servo rotates the arm assembly. We're using a winch servo for more rotation and strength.
 *      a "middle" servo raises the first part of the arm (analogous to a human upper arm). This is a standard servo.
 *      an "end" servo raises the final arm segment (analogous to a human forearm). This is a standard servo.
 */
@TeleOp(name="CurtisArm", group = "Swerve")
// @Disabled
public class CurtisActoArm extends LinearOpMode {

    //scale factors that affect how much each gamepad control stick affects the servo position
    private final double BASE_SCALE_FACTOR = 0.001; //the base is a winch servo: make the stick affect it less so it doesn't move too much
    private final double MIDDLE_SCALE_FACTOR = 0.01;
    private final double END_SCALE_FACTOR = 0.01;

    //what position should each servo start at?
    private final double BASE_STARTING_POSITION = 0.5;
    private final double MIDDLE_STARTING_POSITION = 0.5;
    private final double END_STARTING_POSITION = 0.5;

    //what are the safe ranges for each servo?
    private final double BASE_LIMIT_MIN = 0.0;
    private final double BASE_LIMIT_MAX = 1.0;
    private final double MIDDLE_LIMIT_MIN = 0.0;
    private final double MIDDLE_LIMIT_MAX = 1.0;
    private final double END_LIMIT_MIN = 0.0;
    private final double END_LIMIT_MAX = 1.0;


    //our servos
    private Servo servoBase = null, servoMiddle = null, servoEnd = null;

    //maintain the currently desired position of each servo in a variable,
    //and then have the control sticks change the variable values.
    private double basePosition = BASE_STARTING_POSITION;
    private double middlePosition = MIDDLE_STARTING_POSITION;
    private double endPosition = END_STARTING_POSITION;


    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while (opModeIsActive()) {

            //Base: left stick x controls rotation of base servo.
            //      left (-1) goes clockwise, right (1) goes counterclockwise
            updateBasePosition(gamepad1.left_stick_x);

            //Middle: left stick y controls rotation of middle servo.
            //        up (-1) causes arm to go up, down (1) causes arm to go down
            updateMiddlePosition(gamepad1.left_stick_y);

            //End: right stick y controls rotation of middle servo.
            //     up (-1) causes arm to go up, down (1) causes arm to go down
            updateEndPosition(gamepad1.right_stick_y);


            //reset to starting position
            if (gamepad1.a)
            {
                resetArmPosition();
            }

            //telemetry.addData("base", basePosition);
            //telemetry.addData("middle", middlePosition);
            //telemetry.addData("end", endPosition);
            //telemetry.update();

            idle();
        }
    }

    private void initializeHardware() {
        // Initialize hardware and other important things
        servoBase = hardwareMap.servo.get("base");
        servoMiddle = hardwareMap.servo.get("middle");
        servoEnd = hardwareMap.servo.get("end");

        //initialize starting positions of servos
        resetArmPosition();
    }


    // *****  arm controls *****

    //Base: left stick x controls rotation of base servo.
    //      left (-1) goes clockwise, right (1) goes counterclockwise
    private void updateBasePosition(double offset) {
        if (offset == 0) return;

        basePosition += offset * BASE_SCALE_FACTOR;

        if (basePosition < BASE_LIMIT_MIN) basePosition = BASE_LIMIT_MIN;
        if (basePosition > BASE_LIMIT_MAX) basePosition = BASE_LIMIT_MAX;

        servoBase.setPosition(basePosition);
    }

    //Middle: left stick y controls rotation of middle servo.
    //        up (-1) causes arm to go up, down (1) causes arm to go down
    private void updateMiddlePosition(double offset) {
        if (offset == 0) return;

        middlePosition += offset * MIDDLE_SCALE_FACTOR;

        if (middlePosition < MIDDLE_LIMIT_MIN) middlePosition = MIDDLE_LIMIT_MIN;
        if (middlePosition > MIDDLE_LIMIT_MAX) middlePosition = MIDDLE_LIMIT_MAX;

        servoMiddle.setPosition(middlePosition);
    }

    //End: right stick y controls rotation of middle servo.
    //     up (-1) causes arm to go up, down (1) causes arm to go down
    private void updateEndPosition(double offset) {
        if (offset == 0) return;

        endPosition -= offset * END_SCALE_FACTOR;

        if (endPosition < END_LIMIT_MIN) endPosition = END_LIMIT_MIN;
        if (endPosition > END_LIMIT_MAX) endPosition = END_LIMIT_MAX;

        servoEnd.setPosition(endPosition);
    }

    private void resetArmPosition()
    {
        basePosition = BASE_STARTING_POSITION;
        middlePosition = MIDDLE_STARTING_POSITION;
        endPosition = END_STARTING_POSITION;

        servoBase.setPosition(basePosition);
        servoMiddle.setPosition(middlePosition);
        servoEnd.setPosition(endPosition);
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
