package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;


/**
 * Program used to control Drive-A-Bots.
 * This can be a good reference for drive controls.
 */
//@Autonomous(name="JustinAutonomous", group = "Swerve")
// @Disabled

public class JustinAutonomous extends LinearOpMode
{
    DcMotor motorFront = null;
    DcMotor motorBack = null;
    DcMotor motorLeft = null;
    DcMotor motorRight = null;


    // HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException

    {
        // Initialize hardware and other important things
        initializeRobot();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        forwards(10, 3, .2);
        backwards(10, 3, .2);
        right(10, 3, .2);
        left(10, 3, .2);
        forwardsdiagonalright(10, 3, .2);
        backwardsdiagonalright(10, 3, .2);
        forwardsdiagonalleft(10, 3, .2);
        backwardsdiagonalleft(10, 3, .2);


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // encoderDrive(DRIVE_SPEED, 30, 30, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        // encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        // encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void forwards(double forwardInches, double timeout, double speed)

    {
        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;



        newTargetBL = motorLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetBR = motorRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);


        motorLeft.setTargetPosition(newTargetBL);
        motorRight.setTargetPosition(newTargetBR);

        runtime.reset();
        motorLeft.setPower(Math.abs(speed));
        motorRight.setPower(Math.abs(speed));

        // wait until the motors reach the position
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motorLeft.isBusy() && motorRight.isBusy()));

        // stop the motors
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }


    public void backwards(double x, double timeout, double speed)

    {
        forwards(-x, 3, 0.2);
    }


    public void right(double forwardInches, double timeout, double speed)

    {
        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;


        newTargetFL = motorFront.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetFR = motorBack.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);

        motorFront.setTargetPosition(newTargetFL);
        motorBack.setTargetPosition(newTargetFR);

        runtime.reset();
        motorFront.setPower(Math.abs(speed));
        motorBack.setPower(Math.abs(speed));

        // wait until the motors reach the position
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motorFront.isBusy() && motorBack.isBusy()));

        // stop the motors
        motorFront.setPower(0);
        motorBack.setPower(0);

    }


    public void left(double y, double timeout, double speed)

    {
        right(-y, 3, 0.2);
    }


    public void forwardsdiagonalright(double forwardInches, double timeout, double speed)

    {
        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;


        newTargetFL = motorFront.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetFR = motorBack.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetBL = motorLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetBR = motorRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);

        motorFront.setTargetPosition(newTargetFL);
        motorBack.setTargetPosition(newTargetFR);
        motorLeft.setTargetPosition(newTargetBL);
        motorRight.setTargetPosition(newTargetBR);

        runtime.reset();
        motorFront.setPower(Math.abs(speed));
        motorBack.setPower(Math.abs(speed));
        motorLeft.setPower(Math.abs(speed));
        motorRight.setPower(Math.abs(speed));

        // wait until the motors reach the position
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motorFront.isBusy() && motorBack.isBusy() && motorLeft.isBusy() && motorRight.isBusy()));

        // stop the motors
        motorFront.setPower(0);
        motorBack.setPower(0);
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }


    public void backwardsdiagonalright(double z, double timeout, double speed)

    {
        forwardsdiagonalleft(-z, 3, 0.2);
    }


    public void forwardsdiagonalleft(double forwardInches, double timeout, double speed)

    {
        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;


        newTargetFL = motorFront.getCurrentPosition() + (int) (COUNTS_PER_INCH * -forwardInches);
        newTargetFR = motorBack.getCurrentPosition() + (int) (COUNTS_PER_INCH * -forwardInches);
        newTargetBL = motorLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetBR = motorRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);

        motorFront.setTargetPosition(newTargetFL);
        motorBack.setTargetPosition(newTargetFR);
        motorLeft.setTargetPosition(newTargetBL);
        motorRight.setTargetPosition(newTargetBR);

        runtime.reset();
        motorFront.setPower(Math.abs(speed));
        motorBack.setPower(Math.abs(speed));
        motorLeft.setPower(Math.abs(speed));
        motorRight.setPower(Math.abs(speed));

        // wait until the motors reach the position
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motorFront.isBusy() && motorBack.isBusy() && motorLeft.isBusy() && motorRight.isBusy()));

        // stop the motors
        motorFront.setPower(0);
        motorBack.setPower(0);
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }


    public void backwardsdiagonalleft(double a, double timeout, double speed)

    {
        forwardsdiagonalright(-a, 3, 0.2);
    }


    public void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        motorFront = hardwareMap.dcMotor.get("motorFront");
        motorBack = hardwareMap.dcMotor.get("motorBack");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        // We're not using encoders, so tell the motor controller
        motorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       motorFront.setDirection(DcMotor.Direction.REVERSE);
       motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFront.setPower(0);
        motorBack.setPower(0);
        motorLeft.setPower(0);
        motorRight.setPower(0);

        //Set up telemetry data
        configureDashboard();
    }




    public void configureDashboard()
    {
        telemetry.addLine()
                .addData("Power | FrontLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFront.getPower());
                    }
                })
                .addData("Power | FrontRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBack.getPower());
                    }
                })
                .addData("Power | BackLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorLeft.getPower());
                    }
                })
                .addData("Power | BackRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorRight.getPower());
                    }
                });



    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
