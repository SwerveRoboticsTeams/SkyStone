package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

abstract public class MasterOpMode extends LinearOpMode
{
    // Remembers whether the grabber is open or closed.
    boolean isGrabberOpen = true;
    // Remembers whether the foundation servos are open or closed.
    boolean areFoundationServosOpen = true;
    // Tells us what drive mode the lift motor is in; by default, we use RUN_USING_ENCODER
    boolean isRunToPosMode = false;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    // todo Adjust
    // Distance (in inches) that we want to start collecting after rotating collector.
    int collectionDistance = 12;

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
    Servo foundationServoLeft;
    Servo foundationServoRight;
    //----------------------------------------------------------------

    // Declare filters.  We currently have PID for turning and encoder navigation.------------------
    PIDFilter rotationFilter;
    PIDFilter translationFilter;

    // Stores orientation of robot
    double currentAngle = 0.0;

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
    //                                             y = 0 + 0.5x + 0 + 0.5x^3
    Polynomial stickCurve = new Polynomial(new double[]{0, 0.5, 0, 0.5});

    // Variable that keeps track of the height of our tower. It tells autonomous and semi-autonomous drive how high the current tower is.
    int towerHeight = 0;


    public void initialize()
    {
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

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

        foundationServoLeft = hardwareMap.servo.get("foundationServoLeft");
        foundationServoRight = hardwareMap.servo.get("foundationServoRight");

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
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberServo.setPosition(Constants.GRABBER_OPEN);
        parallelServo.setPosition(Constants.PARALLEL_SERVO_INIT);
        foundationServoLeft.setPosition(Constants.FOUNDATION_SERVO_LEFT_OPEN);
        foundationServoRight.setPosition(Constants.FOUNDATION_SERVO_RIGHT_OPEN);


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

        // Initialize PID filters
        rotationFilter = new PIDFilter(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);
        translationFilter = new PIDFilter(Constants.TRANSLATION_P, Constants.TRANSLATION_I, Constants.TRANSLATION_D);

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
         Motor powers might be set above 1 (e.g., x + y = 1.0 and w = 1.0), so we must scale all of
         the powers to ensure they are proportional and within the range {-1.0, 1.0}
        */
        double powScalar = SequenceUtilities.getLargestMagnitude(new double[]
                {powerFL, powerFR, powerBL, powerBR});
        /*
         However, powScalar should only be applied if it is greater than 1. Otherwise, we could
         unintentionally increase powers or even divide by 0
        */
        if (powScalar > 1)
        {
            powerFL /= powScalar;
            powerFR /= powScalar;
            powerBL /= powScalar;
            powerBR /= powScalar;
        }

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    // Uses encoders to make the robot drive to a specified relative position.  Also makes use of the
    // imu to keep the robot at a constant heading during navigation.
    // Finally, we use the boolean isCollecting to give us the option to control the collector
    // while the robot is navigating.
    // **Note:  initDeltaX/Y are in inches.
    void navigateUsingEncoders(double initDeltaX, double initDeltaY, double maxPower, boolean isCollecting)
    {
        // Variables set every loop-------------------
        double deltaX = initDeltaX;
        double deltaY = initDeltaY;
        double initHeading = getAngularOrientationWithOffset();

        double driveAngle;
        double drivePower;
        double rotationPower;

        // Find distance and angle between robot and its destination
        double distanceToTarget = calculateDistance(deltaX, deltaY);

        currentAngle = getAngularOrientationWithOffset();
        double headingDiff = normalizeRotationTarget(initHeading, currentAngle);
        //---------------------------------------------

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Check to see if robot has arrived at destination within angle and position tolerances
        while (((distanceToTarget > Constants.POSITION_TOLERANCE_IN) || (headingDiff > Constants.ANGLE_TOLERANCE_DEG)) && opModeIsActive())
        {
            deltaX = initDeltaX - Constants.IN_PER_ANDYMARK_TICK * (motorFL.getCurrentPosition() -
                    motorBL.getCurrentPosition() + motorFR.getCurrentPosition() - motorBR.getCurrentPosition()) / (4 /* Math.sqrt(2)*/);
            deltaY = initDeltaY - Constants.IN_PER_ANDYMARK_TICK * (motorFL.getCurrentPosition() +
                    motorBL.getCurrentPosition() - motorFR.getCurrentPosition() - motorBR.getCurrentPosition()) / 4;

            // Calculate how far off robot is from its initial heading
            currentAngle = getAngularOrientationWithOffset();
            headingDiff = normalizeRotationTarget(initHeading, currentAngle);

            // Recalculate drive angle and distance remaining every loop
            distanceToTarget = calculateDistance(deltaX, deltaY);
            driveAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

            // todo - signs should be properly accounted for.
            // Transform position and heading diffs to linear and rotation powers using filters----
            translationFilter.roll(-distanceToTarget);
            drivePower = translationFilter.getFilteredValue();

            // Ensure robot doesn't approach target position too slowly
            if (Math.abs(drivePower) < Constants.MINIMUM_DRIVE_POWER)
            {
                drivePower = Math.signum(drivePower) * Constants.MINIMUM_DRIVE_POWER;
            }
            // Ensure robot doesn't ever drive faster than we want it to
            else if (Math.abs(drivePower) > maxPower)
            {
                drivePower = Math.signum(drivePower) * maxPower;
            }

            // todo - signs should be properly accounted for.
            // Additional factor is necessary to ensure turning power is large enough
            rotationFilter.roll(-headingDiff);
            rotationPower = rotationFilter.getFilteredValue();
            //-------------------------------------------------------------------------------------

            // If it has been specified, collect while navigating.
            if (isCollecting)
            {
                // If we have navigated less than collectionDistance, rotate the stone.
                // Otherwise, collect the stone.
                if (Math.abs(deltaY) > collectionDistance)
                    runCollector(true, true);
                else
                    runCollector(true, false);
            }

            driveMecanum(driveAngle, drivePower, rotationPower);

            telemetry.addData("Encoder Diff x: ", deltaX);
            telemetry.addData("Encoder Diff y: ", deltaY);
            telemetry.addData("Heading Diff: ", headingDiff);
            telemetry.update();
            idle();
        }

        // Turn off drive motors
        stopDriveMotors();

        // Turn off collector if we were running it.
        if (isCollecting)
        {
            pauseWhileUpdating(2.0);

            collectorLeft.setPower(0);
            collectorRight.setPower(0);
        }
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

    // Tell the robot to turn to a specified angle.  We can also limit the motor power while turning.
    public void turnTo(double targetAngle, double maxPower)
    {
        double turningPower;
        currentAngle = getAngularOrientationWithOffset();
        double angleDiff = normalizeRotationTarget(targetAngle, currentAngle);

        // Robot only stops turning when it is within angle tolerance
        while (Math.abs(angleDiff) >= Constants.ANGLE_TOLERANCE_DEG && opModeIsActive())
        {
            currentAngle = getAngularOrientationWithOffset();

            // Give robot raw value for turning power
            angleDiff = normalizeRotationTarget(targetAngle, currentAngle);

            // Send raw turning power through PID filter to adjust range and minimize oscillation
            rotationFilter.roll(angleDiff);
            turningPower = rotationFilter.getFilteredValue();

            // Make sure turningPower doesn't go above maximum power
            if (Math.abs(turningPower) > maxPower)
            {
                turningPower = maxPower * Math.signum(turningPower);
            }

            // Makes sure turningPower doesn't go below minimum power
            if (Math.abs(turningPower) < Constants.MINIMUM_TURNING_POWER)
            {
                turningPower = Math.signum(turningPower) * Constants.MINIMUM_TURNING_POWER;
            }

            // Turns robot
            driveMecanum(0.0, 0.0, -turningPower);

            telemetry.addData("angleDiff: ", angleDiff);
            telemetry.addData("Turning Power: ", turningPower);
            telemetry.addData("Orientation: ", currentAngle);
            telemetry.update();
            idle();
        }

        stopDriveMotors();
    }


    // todo Make sure this works
    // General method for driving collector (auto and TeleOp)
    public void runCollector(boolean isCollectingIn, boolean isRotatingStone)
    {
        // Allows us to change direction of collector
        double powerSign = 1.0;

        // Reverse collector if we intend to spit out
        if (!isCollectingIn)
            powerSign = -1.0;

        if (isRotatingStone)    // We are trying to spin stone for easier collection
        {
            collectorLeft.setPower(powerSign * Constants.COLLECTOR_ROTATE_POWER);
            collectorRight.setPower(powerSign * Constants.COLLECTOR_ROTATE_POWER);
        } else    // We are collecting normally
        {
            collectorLeft.setPower(powerSign * Constants.COLLECTOR_POWER);
            collectorRight.setPower(powerSign * -Constants.COLLECTOR_POWER);
        }
    }


    // Toggle position of both foundation servos between open and closed.
    public void toggleFoundationServos()
    {
        if (areFoundationServosOpen)
        {
            foundationServoLeft.setPosition(Constants.FOUNDATION_SERVO_LEFT_CLOSED);
            foundationServoRight.setPosition(Constants.FOUNDATION_SERVO_RIGHT_CLOSED);
        } else
        {
            foundationServoLeft.setPosition(Constants.FOUNDATION_SERVO_LEFT_OPEN);
            foundationServoRight.setPosition(Constants.FOUNDATION_SERVO_RIGHT_OPEN);
        }
        areFoundationServosOpen = !areFoundationServosOpen;

        //telemetry.addData("foundationServoLeft Position: ", foundationServoLeft.getPosition());
        //telemetry.addData("foundationServoRight Position: ", foundationServoRight.getPosition());
    }


    // Toggle position of grabber between open and closed.
    public void toggleGrabber()
    {
        if (isGrabberOpen)
        {
            grabberServo.setPosition(Constants.GRABBER_CLOSED);
        } else
        {
            grabberServo.setPosition(Constants.GRABBER_OPEN);
        }
        isGrabberOpen = !isGrabberOpen;
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
        while (Math.abs(rawAngle) > 180)
        {
            rawAngle -= Math.signum(rawAngle) * 360;
        }

        return rawAngle;
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
