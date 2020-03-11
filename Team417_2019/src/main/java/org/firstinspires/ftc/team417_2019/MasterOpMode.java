package org.firstinspires.ftc.team417_2019;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.team417_2019.Resources.Constants;
import org.firstinspires.ftc.team417_2019.Resources.FIRFilter;
import org.firstinspires.ftc.team417_2019.Resources.PIDFilter;
import org.firstinspires.ftc.team417_2019.Resources.Polynomial;
import org.firstinspires.ftc.team417_2019.Resources.Toggler;

import java.util.Locale;

abstract public class MasterOpMode extends LinearOpMode
{

    public Robot robot = new Robot(this);
    // Declare drive motors
    public DcMotor motorFL = null; // hub 2 port 0
    public DcMotor motorFR = null; // hub 1 port 0
    public DcMotor motorBL = null; // hub 2 port 1
    public DcMotor motorBR = null; // hub 1 port 1
    public DcMotor liftMotor1 = null;
    public DcMotor liftMotor2 = null;
    public DcMotor collectorMotorLeft = null;
    public DcMotor collectorMotorRight = null;
    // Declare servos
    // hub 2 port 0
    public Servo leftFoundationPuller = null;
    // hub 2 port 1
    public Servo rightFoundationPuller = null;
    public Servo linkageServo = null;
    public Servo grabberServo = null;


    // For movement using Vuforia
    public BNO055IMU imu;
    Orientation angles;
    OpenGLMatrix vuMark;
    VectorF translation;


    // Declare constants
    // movement constants
    static final double COUNTS_PER_MOTOR_REV = 1120; // 40:1 motor
    static final double DRIVE_GEAR_REDUCTION = 16.0 / 24.0; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_MM = COUNTS_PER_INCH / 25.4;

    final double ROBOT_DIAMETER_MM = 20.5 * 25.4;   // diagonal 20.5 inch FL to BR and FR to BL
    static final double INIT_REV_POS = 0.7; // the initial position is inside the robot, should happen in init
    static final double REV_INCREMENT = 0.0085; // this is how much the REV servo wrist moves per flick of the joystick

    static final int MAX_CORE_POS = 790;
    static final int MIN_CORE_POS = -15;

    // change and edit these
    // servo init and low positions
    static final double MARKER_LOW = 0.5;
    static final double MARKER_HIGH = 0.0;

    // declare color sensor variables
    float hsvValues[] = {0F,0F,0F};
    float hsvLeft[] = {0F,0F,0F};
    float hsvRight[] = {0F,0F,0F};

    PIDFilter turnFilter;
    PIDFilter moveFilter;
    FIRFilter accelerationFilter;
    int filterLength = 10;

    Toggler grabber = new Toggler(Constants.grabberServoIn, Constants.grabberServoOut, grabberServo);
    Toggler leftPuller = new Toggler(Constants.leftFoundationPullerIn, Constants.leftFoundationPullerOut, leftFoundationPuller);
    Toggler rightPuller = new Toggler(Constants.rightFoundationPullerIn, Constants.rightFoundationPullerOut, rightFoundationPuller);
    Toggler linkage = new Toggler(Constants.linkageServoIn, Constants.linkageServoOut, linkageServo);

    public void initializeHardware()
    {
        turnFilter = new PIDFilter(0.02, 0, 0.02);
        moveFilter = new PIDFilter(0.04, 0, 0);
        // weights for weighted average
        double[] filterCoefficients = {1};
        accelerationFilter = new FIRFilter(new Polynomial(filterCoefficients),filterLength);
        accelerationFilter.values = new double[filterLength];


        // Initialize motors to be the hardware motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        // not currently in configuration ( need to be configured on the robot)
        leftFoundationPuller = hardwareMap.servo.get("leftFoundationPuller");
        rightFoundationPuller = hardwareMap.servo.get("rightFoundationPuller");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse front and back right motors just for TeleOp
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        collectorMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        collectorMotorRight.setDirection(DcMotor.Direction.FORWARD);


        // set motor power to 0
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
        collectorMotorLeft.setPower(0);
        collectorMotorRight.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Adaf'" +
                "ruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.log().setCapacity(8);


    } //-----------------------END OF INITIALIZATION SOFTWARE------------------------


    // normalizing the angle to be between -180 to 180
    public double adjustAngles(double angle)
    {
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
    }

    // wait a number of milliseconds
    public void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while(((System.nanoTime() - initialTime)/1000/1000 < t) && opModeIsActive())
        {
            idle();
        }
    }

    public float DetermineLineHue(ColorSensor sensorColor)
    {
        // Convert the RGB values to HSV values
        Color.RGBToHSV((sensorColor.red() * 255) / 800, (sensorColor.green() * 255) / 800, (sensorColor.blue() * 255) / 800, hsvValues);
        return hsvValues[0]; // return the hue (index 0) of HSV
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // get stick angles
    public double getLeftStickAngle(Gamepad gamepad)
    {
        return Math.atan2(-gamepad.left_stick_y, gamepad.left_stick_x);
    }
    public double getRightStickAngle(Gamepad gamepad)
    {
        return Math.atan2(-gamepad.right_stick_y, gamepad.right_stick_x);
    }

    // get stick magnitudes
    public double getLeftStickMagnitude(Gamepad gamepad)
    {
        return Math.sqrt(Math.pow(gamepad.left_stick_x, 2) + Math.pow(gamepad.left_stick_y, 2));
    }
    public double getRightStickMagnitude(Gamepad gamepad)
    {
        return Math.sqrt(Math.pow(gamepad.right_stick_x, 2) + Math.pow(gamepad.right_stick_y, 2));
    }

    public void mecanumDrive(double angle, double drivePower, double rotationalPower) {

        double x = drivePower * Math.cos(angle/* + 90*/);
        double y = drivePower * Math.sin(angle/* + 90*/);

        double frontLeft = y + x + rotationalPower;
        double frontRight = y - x - rotationalPower;
        double backLeft = y - x + rotationalPower;
        double backRight = y + x - rotationalPower;

        // get the largest power
        double powerScalar = returnLargestValue(new double[]{frontLeft, frontRight, backLeft, backRight});

        // scale the power to keep the wheels proportional and between the range of -1 and 1
        if (powerScalar > 1) {
            frontLeft /= powerScalar;
            frontRight /= powerScalar;
            backLeft /= powerScalar;
            backRight /= powerScalar;
        }

        motorFL.setPower(frontLeft);
        motorFR.setPower(frontRight);
        motorBL.setPower(backLeft);
        motorBR.setPower(backRight);
    }

    // takes array of doubles and returns the largest value
    public double returnLargestValue(double[] numberArray){
        double max = Double.MIN_VALUE;
        for ( double num : numberArray ) {
            if (num > max) {
                max = num;
            }
        }
        return max;
    }

    public void runCollector(double power, boolean slowMode) {
        if (!slowMode) {
            collectorMotorRight.setPower(power);
            collectorMotorLeft.setPower(power);
        } else {
            collectorMotorLeft.setPower(power * Constants.collectorSlowModeMultiplier);
            collectorMotorRight.setPower(power * Constants.collectorSlowModeMultiplier);
        }

    }

}
