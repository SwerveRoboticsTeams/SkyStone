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
import java.util.Locale;

abstract public class MasterOpMode extends LinearOpMode
{
    // Declare drive motors
    DcMotor motorFL = null; // hub 2 port 0
    DcMotor motorFR = null; // hub 1 port 0
    DcMotor motorBL = null; // hub 2 port 1
    DcMotor motorBR = null; // hub 1 port 1
    // hub 2 port 2
    public DcMotor core2 = null;
    // hub 1 port 3
    DcMotor arm1 = null;
    // hub 2 port 3
    DcMotor arm2 = null;
    // hub 2 port 3
    Servo mainWristServo = null;
    // hub 2 port 5
    Servo smallGrabber  = null;
    // hub 2 port 0
    public Servo leftFoundationPuller = null;
    // hub 2 port 1
    public Servo rightFoundationPuller = null;

    // For movement using Vuforia
    public BNO055IMU imu;
    Orientation angles;
    OpenGLMatrix vuMark;
    VectorF translation;

    // Declare constants
    // movement constants
    static final double COUNTS_PER_MOTOR_REV = 1120; // 40:1 motor
    static final double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_MM = COUNTS_PER_INCH / 25.4;
    static final double SCALE_OMNI = 1.0 /*1.41 todo should be sqrt(2) but 1 works*/;

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



    public void initializeHardware()
    {
        // Initialize motors to be the hardware motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        core2 = hardwareMap.dcMotor.get("core2");

        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");

        // not currently in configuration ( need to be configured on the robot)
        mainWristServo = hardwareMap.servo.get("mainWristServo");
        smallGrabber = hardwareMap.servo.get("smallGrabber");
        leftFoundationPuller = hardwareMap.servo.get("leftFoundationPuller");
        rightFoundationPuller = hardwareMap.servo.get("rightFoundationPuller");

        core2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        core2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse front and back right motors just for TeleOp
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        //motorFL.setMode();

        // set motor power to 0
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        core2.setPower(0);

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

        double x = drivePower * Math.cos(angle + 90);
        double y = drivePower * Math.sin(angle + 90);

        // todo swap order of x and y - test on robot
        double frontLeft = y - x + rotationalPower;
        double frontRight = y + x - rotationalPower;
        double backLeft = y + x + rotationalPower;
        double backRight = y - x - rotationalPower;

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

}
