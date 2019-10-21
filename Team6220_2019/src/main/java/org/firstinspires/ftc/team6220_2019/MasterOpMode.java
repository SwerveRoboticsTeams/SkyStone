package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

abstract public class MasterOpMode extends LinearOpMode
{
    // Boolean for team color.
    boolean isRed;

    // Create instance of VuforiaResources to be used for image tracking.  We need to pass in this
    // opMode to be able to use some functionalities in that class.
    VuforiaResources vRes = new VuforiaResources(this);

    // Declare hardware devices---------------------------------------
    BNO055IMU imu;

    DcMotor motorFL, motorFR, motorBL, motorBR;
    DcMotor collectorLeft, collectorRight;
    DcMotor liftMotor;

    Servo grabberServo;
    Servo parallelServo;
    //----------------------------------------------------------------

    // Create drivers
    DriverInput driver1;
    DriverInput driver2;

    // Create a list of tasks to accomplish in order
    List<ConcurrentOperation> callback = new ArrayList<>();

    ElapsedTime timer = new ElapsedTime();
    double lTime = 0;

    // Allows us to use global rotation coordinates by specifying starting orientation.
    double startingOrientation;

    /*
    Polynomial for adjusting input from joysticks to allow for ease of driving.  A polynomial that
    is concave up in the 1st quadrant is preferable to a 1st degree polynomial, since a driver
    generally needs more control in the low speed range than the high range.
   */
    //                                             y = 0 + 0.25x + 0 + 0.75x^3
    Polynomial stickCurve = new Polynomial(new double[]{0, 0.25, 0, 0.75});


    public void initialize()
    {
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        // Drive motor initialization--------------------------------------
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        collectorLeft = hardwareMap.dcMotor.get("collectorLeft");
        collectorRight = hardwareMap.dcMotor.get("collectorRight");

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        grabberServo = hardwareMap.servo.get("grabberServo");
        parallelServo = hardwareMap.servo.get("parallelServo");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // todo Change once encoder is working
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberServo.setPosition(Constants.GRABBER_OPEN);
        parallelServo.setPosition(Constants.PARALLEL_SERVO_INIT);


        stopDriveMotors();
        //-----------------------------------------------------------------

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

        // Add necessary items to callback---------------
        callback.add(driver1);
        callback.add(driver2);
    }


    public void driveMecanum(double driveAngle, double drivePower, double w)
    {
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
        double powScalar = SequenceUtilities.getLargestMagnitude(new double[]
                {powerFL, powerFR, powerBL, powerBR});
        /*
         However, powScalar should only be applied if it is greater than 1. Otherwise, we could
         unintentionally increase powers or even divide by 0
        */
//        if (powScalar > 1)
//        {
//            powerFL /= powScalar;
//            powerFR /= powScalar;
//            powerBL /= powScalar;
//            powerBR /= powScalar;
//        }

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    // The only difference between autonomousDriveMecanum and driveMecanum is that autonomousDriveMecanum
    // contains an adjustment for angle.
    public void autonomousDriveMecanum(double driveAngle, double drivePower, double w)
    {
        // Adjustment to account for different axis reference frames.
        driveAngle += 90;

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
        double powScalar = SequenceUtilities.getLargestMagnitude(new double[]
                {powerFL, powerFR, powerBL, powerBR});
        /*
         However, powScalar should only be applied if it is greater than 1. Otherwise, we could
         unintentionally increase powers or even divide by 0
        */
//        if (powScalar > 1)
//        {
//            powerFL /= powScalar;
//            powerFR /= powScalar;
//            powerBL /= powScalar;
//            powerBR /= powScalar;
//        }

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    // Note:  not in use
    // Other opmodes must go through this method to prevent others from unnecessarily changing startingOrientation
    void setRobotStartingOrientation(double newValue)
    {
        startingOrientation = newValue;
    }

    // Prevents angle differences from being outside the range -180 to 180 degrees
    public double normalizeRotationTarget(double finalAngle, double initialAngle)
    {
        double diff = finalAngle - initialAngle;

        return normalizeAngle(diff);
    }


    // Prevents a single angle from being outside the range -180 to 180 degrees
    public double normalizeAngle(double rawAngle)
    {
        return rawAngle % 360 - 180;
    }


    // Takes into account startingOrientation to utilize global orientation
    double getAngularOrientationWithOffset()
    {
        double correctedHeading = normalizeAngle(imu.getAngularOrientation().firstAngle + startingOrientation);

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

        while (opModeIsActive() && (time > 0))
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
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}
