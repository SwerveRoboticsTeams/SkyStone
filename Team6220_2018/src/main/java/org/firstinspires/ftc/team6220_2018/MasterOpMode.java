package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

abstract public class MasterOpMode extends LinearOpMode
{
    // Uses the robot's starting orientation to modify imu output and create global heading
    private double headingOffset = 0;

    /*
     Polynomial for adjusting input from joysticks to allow for ease of driving.  A polynomial that
     is concave up in the 1st quadrant is preferable to a 1st degree polynomial, since a driver
     generally needs more control in the low speed range than the high range.
    */
    //                                             y = 0 + 0.25x + 0 + 0.75x^3
    Polynomial stickCurve = new Polynomial(new double[]{ 0, 0.5, 0, 0.5});

    // Contains useful vuforia methods
    VuforiaHelper vuforiaHelper;

    ElapsedTime timer = new ElapsedTime();
    double lTime = 0;
    ElapsedTime collectorLoopTimer = new ElapsedTime();

    DriverInput driver1;
    DriverInput driver2;
    // Optical encoder in front of slotted wheel that allows our Vex motor to tell its position
    // in the absence of a built-in encoder.
    ConcurrentDigitalDevice collectorChannel;
    DigitalChannel collectorEncoder;
    CollectorMechanism collectorMech;

    // Classes that encapsulate distinct hardware systems
        //CollectorMechanism armMechanism;

    // todo Implement new framework PID when it becomes available
    // Declare filters.  We currently have PID for turning and encoder navigation.------------------
        PIDFilter rotationFilter;
        PIDFilter translationFilter;
        //PIDFilter armFilter;
    //----------------------------------------------------------------------------------------------

    // Declare hardware devices---------------------------------------
    BNO055IMU imu;

    // Motors----------------------
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor motorHanger;
    DcMotor motorArmLeft;
    DcMotor motorArmRight;
     //-----------------------------

     // Servos----------------------
    Servo servoHanger;
    CRServo motorCollector;
    Servo servoMarker;
     //-----------------------------
    //-----------------------------------------------------------------

    // Servo togglers

    // Booleans that allow us to choose what parts of the robot we are and aren't using in each OpMode
    public boolean isDriveTrainAttached = true;
    public boolean isHangerAttached = true;
    public boolean isArmMechAttached = true;

    // Create a list of tasks to accomplish in order
    List<ConcurrentOperation> callback = new ArrayList<>();


    public void initializeRobot()
    {
         // Check to see what parts of the robot are attached.  Some programs (e.g., autonomous and -------------------------
         // tests) may want to ignore parts of the robot that don't need to be used
        if (isDriveTrainAttached)
        {
            // Drive motors
            motorFL = hardwareMap.dcMotor.get("motorFrontLeft");
            motorFR = hardwareMap.dcMotor.get("motorFrontRight");
            motorBL = hardwareMap.dcMotor.get("motorBackLeft");
            motorBR = hardwareMap.dcMotor.get("motorBackRight");

            // Set motor attributes and behaviors------------------------------
            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);

            servoMarker = hardwareMap.servo.get("servoMarker");
            servoMarker.setPosition(Constants.SERVO_MARKER_RETRACTED);
            //-------------------------------------------------------------------
        }
        if (isHangerAttached)
        {
            motorHanger = hardwareMap.dcMotor.get("motorHanger");

            motorHanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Rest of initialization is at beginning of TeleOp.

            servoHanger = hardwareMap.servo.get("servoHanger");
        }
        if(isArmMechAttached)
        {
            motorCollector = hardwareMap.crservo.get("motorCollector");
            motorCollector.setPower(0);

            motorArmLeft = hardwareMap.dcMotor.get("motorArmLeft");
            //motorArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorArmLeft.setPower(0);

            motorArmRight = hardwareMap.dcMotor.get("motorArmRight");
            //motorArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorArmRight.setPower(0);
            // Rest of initialization is at beginning of TeleOp.
        }
        //--------------------------------------------------------------------------------------------------------------


        // todo REV imu can occasionally taking a long time to initialize or even fail to do so; why is this?
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Initialize PID filters
        rotationFilter = new PIDFilter(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);
        translationFilter = new PIDFilter(Constants.TRANSLATION_P, Constants.TRANSLATION_I, Constants.TRANSLATION_D);
        //armFilter = new PIDFilter(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);


        // Initialize robot mechanism classes----------------------------
        //armMechanism = new CollectorMechanism(this);
        //---------------------------------------------------------------
        collectorEncoder = hardwareMap.digitalChannel.get("collectorEncoder");
        collectorEncoder.setMode(DigitalChannel.Mode.INPUT);

        // Objects that must be updated each callback
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);
        collectorChannel = new ConcurrentDigitalDevice(collectorEncoder);
        collectorMech = new CollectorMechanism(this, driver2, motorCollector, collectorChannel, collectorLoopTimer);

        // Add necessary items to callback---------------
        callback.add(driver1);
        callback.add(driver2);
        callback.add(collectorChannel);
        callback.add(collectorMech);
        //-----------------------------------------------


        // Note:  not in use
        for (ConcurrentOperation item : callback)
        {
            item.initialize(hardwareMap);
        }
    }

    void powerArm(double drivePower)
    {
        if(!isArmMechAttached)
        {
            telemetry.addLine("Collector/arm is not attached!");
            telemetry.update();
            return;
        }

        double power = drivePower;
        motorArmLeft.setPower(drivePower);
        motorArmRight.setPower(-drivePower);
    }

    /*
     General method for driving robot.  Input is given as a vector in polar form with
     (r, theta) = (drivePower, driveAngle)

     Table for mecanum drive motor directions (clockwise = positive):

                  FL      FR      BL       BR
     rotate -w    -        -      -        -
     rotate +w    +        +      +        +
     forward      -        +      -        +
     backward     +        -      +        -
     right        -        -      +        +
     left         +        +      -        -
     diag. left   +        0      0        -
     diag. right  -        0      0        +
    */
    void driveMecanum(double driveAngle, double drivePower, double w)
    {
        if(!isDriveTrainAttached)
        {
            telemetry.addLine("Drive is not attached!");
            telemetry.update();
            return;
        }

        // Convert drive angle and power to x and y components
        double y = drivePower * Math.sin(Math.toRadians(driveAngle));
        double x = drivePower * Math.cos(Math.toRadians(driveAngle));

        // Signs for x, y, and w are based on the motor configuration and inherent properties of mecanum drive
        double powerFL = -x - y + w;
        double powerFR = -x + y + w;
        double powerBL = x - y + w;
        double powerBR = x + y + w;

        // Scale powers-------------------------
        /*
         Motor powers might be set above 1 (e.g., x + y = 1 and w = -0.8), so we must scale all of
         the powers to ensure they are proportional and within the range {-1.0, 1.0}
        */
        double powScalar = SequenceUtilities.getLargestMagnitude(new double[]{powerFL, powerFR, powerBL, powerBR});
        /*
         However, powScalar should only be applied if it is greater than 1. Otherwise, we could
         unintentionally increase powers or even divide by 0
        */
        if(powScalar < 1)
            powScalar = 1;

        powerFL /= powScalar;
        powerFR /= powScalar;
        powerBL /= powScalar;
        powerBR /= powScalar;
        //--------------------------------------

        // Power motors with corrected inputs
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);

        // todo How to make this not interfere with other telemetry?
        // Telemetry for debugging motor power inputs
        /*
        telemetry.addData("translation power: ", x);
        telemetry.addData("vertical power: ", y);
        telemetry.addData("rotational power: ", w);
        telemetry.update();
        */
    }

    // Allows robot to move sideways while maintaining collector as center of robot.
    void driveMecanumWithCurving(double driveAngle, double drivePower, double w, double powerFLMult, double powerFRMult, double powerBLMult, double powerBRMult)
    {
        if (!isDriveTrainAttached)
        {
            telemetry.addLine("Drive is not attached!");
            telemetry.update();
            return;
        }

        // Convert drive angle and power to x and y components
        double y = drivePower * Math.sin(Math.toRadians(driveAngle));
        double x = drivePower * Math.cos(Math.toRadians(driveAngle));

        // Signs for x, y, and w are based on the motor configuration and inherent properties of mecanum drive
        double powerFL = -x - y + w;
        double powerFR = -x + y + w;
        double powerBL = x - y + w;
        double powerBR = x + y + w;

        // Scale powers-------------------------
        /*
         Motor powers might be set above 1 (e.g., x + y = 1 and w = -0.8), so we must scale all of
         the powers to ensure they are proportional and within the range {-1.0, 1.0}
        */
        double powScalar = SequenceUtilities.getLargestMagnitude(new double[]{powerFL, powerFR, powerBL, powerBR});
        /*
         However, powScalar should only be applied if it is greater than 1. Otherwise, we could
         unintentionally increase powers or even divide by 0
        */
        if (powScalar < 1)
            powScalar = 1;

        powerFL /= powScalar;
        powerFR /= powScalar;
        powerBL /= powScalar;
        powerBR /= powScalar;
        //--------------------------------------

        // Power motors with corrected inputs
        motorFL.setPower(powerFL / powerFLMult);
        motorFR.setPower(powerFR / powerFRMult);
        motorBL.setPower(powerBL / powerBLMult);
        motorBR.setPower(powerBR / powerBRMult);

        // todo How to make this not interfere with other telemetry?
        // Telemetry for debugging motor power inputs
        /*
        telemetry.addData("translation power: ", x);
        telemetry.addData("vertical power: ", y);
        telemetry.addData("rotational power: ", w);
        telemetry.update();
        */
    }


    // Note:  not in use
    // Other opmodes must go through this method to prevent others from unnecessarily changing headingOffset
    void setRobotStartingOrientation(double newValue)
    {
        headingOffset = newValue;
    }


    // Prevents angle differences from being outside the range -180 to 180 degrees
    public double normalizeRotationTarget(double finalAngle, double initialAngle)
    {
        double diff = finalAngle - initialAngle;

        while (Math.abs(diff) > 180)
        {
            diff -= Math.signum(diff) * 360;
        }

        return diff;
    }


    // Prevents a single angle from being outside the range -180 to 180 degrees
    public double normalizeAngle(double rawAngle)
    {
        while (Math.abs(rawAngle) > 180)
        {
            rawAngle -= Math.signum(rawAngle) * 360;
        }

        return rawAngle;
    }


    // Takes into account headingOffset to utilize global orientation
    double getAngularOrientationWithOffset()
    {
        double correctedHeading = normalizeAngle(imu.getAngularOrientation().firstAngle + headingOffset);

        return correctedHeading;
    }


    // Finds distance between 2 points
    double calculateDistance(double dx, double dy)
    {
        double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        return distance;
    }


    // Updates every item with elapsed time at the end of the main loop; ensures that operations
    // based on a timer are executed on time
    public void updateCallback(double eTime)
    {
        for (ConcurrentOperation item : callback)
        {
            item.update(eTime);
        }
    }


    // Note:  time parameter is in seconds
    // Waits for a specified time while giving each hardware system the ability to function simultaneously
    void pauseWhileUpdating(double time)
    {
        lTime = timer.seconds();

        while(opModeIsActive() && (time > 0))
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();
            time -= eTime;

            updateCallback(eTime);
            //telemetry.addData("eTime:", eTime);
            //telemetry.addData("Seconds Remaining:", time);
            //telemetry.update();
            idle();
        }
    }


    void stopDriveMotors()
    {
        if(!isDriveTrainAttached)
        {
            telemetry.addLine("Drive is not attached!");
            telemetry.update();
            return;
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}
