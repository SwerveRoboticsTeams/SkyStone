package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Vec2I;

/**
 * This class includes methods and fields that are useful for generic autonomous OpModes.
 */
abstract public class MasterAutonomous extends MasterOpMode
{
    // Initialize booleans used in runSetup()--------------------------------------------------
    boolean isRedAlliance = true;
    // The amount of time we want to delay at the beginning of the match
    int delayCount = 0;
    // The number of SkyStones we want to score; can take a value from 0 - 2
    int numSkyStones = 1;
    // The position of the SkyStone; initialized by vuforiaAlignWithSkystone()
    int skyStonePos;
    // Whether we want to score the foundation or not
    boolean scoreFoundation = true;
    // Whether we want to park on far or close end of line under alliance Skybridge
    boolean parkClose = true;
    //------------------------------------------------------------------------------------------

    // Initialize angles and distances for various differing setup options----------------------
    double stoneShift = 8; // Centers of stones are 8 in apart
    double robotShift = stoneShift;     // Actual distance the robot moved (will be changed)

    // Change initial orientation based on red / blue alliance.  Red is default
    double initAngle_side = 90;
    // Change some turns based on alliance
    double turnShift = 0;
    // Account for translation being opposite directions relative to field on red / blue alliances
    double robotShiftSign = 1.0;
    // Account for center being shifted in opposite directions on red / blue alliance
    double centerAdjustment = 0;

    //------------------------------------------------------------------------------------------


    // Initializes robot normally in addition to added autonomous functionality (e.g., Vuforia)
    // Also needs to pass in Vuforia use boolean and another boolean (isRunningSetup) that
    // specifies whether we need to use the controller to set up our autonomous.
    public void initialize(boolean isUsingVuforia, boolean isRunningSetup)
    {
        super.initialize(isUsingVuforia);

        if (isRunningSetup)
            runSetup();
    }


    // Allows the 1st driver to decide which autonomous routine should be run using gamepad input
    void runSetup()
    {
        // Accounts for delay between initializing the program and starting TeleOp.
        lTime = timer.seconds();

        // Ensure log can't overflow
        telemetry.log().setCapacity(5);
        telemetry.log().add("Red / Blue = B / X");
        telemetry.log().add("Increase / Decrease Delay = DPad Up / Down");
        telemetry.log().add("Score / Not Score foundation = Toggle Y");
        telemetry.log().add("Increase / Decrease numSkyStones = DPad Left / Right");
        telemetry.log().add("Park Close / Far = Toggle A");
        telemetry.log().add("Press Start to exit setup.");

        boolean settingUp = true;

        while (settingUp && !isStopRequested())
        {
            // Finds the time elapsed each loop.
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            // Select alliance
            if (driver1.isButtonJustPressed(Button.B))
                isRedAlliance = true;
            else if (driver1.isButtonJustPressed(Button.X))
                isRedAlliance = false;

            // Adjust match delay from 0 to 10 seconds
            if (driver1.isButtonJustPressed(Button.DPAD_UP) && delayCount < 10)    // Don't want delay to be to large
                delayCount++;
            else if (driver1.isButtonJustPressed(Button.DPAD_DOWN) && delayCount > 0)   // Also don't want it to be negative
                delayCount--;

            // Adjust whether we score foundation
            if (driver1.isButtonJustPressed(Button.Y))
                scoreFoundation = !scoreFoundation;

            // todo Fix (only shows 1 / 2)
            // Adjust number of SkyStones scored from 0 to 2
            if (driver1.isButtonJustPressed(Button.DPAD_RIGHT) && numSkyStones < 2)    // Increase number of SkyStones (2 max)
                numSkyStones++;
            else if (driver1.isButtonJustPressed(Button.DPAD_RIGHT) && numSkyStones > 0)    // Increase number of SkyStones (2 max)
                numSkyStones--;

            // Switch between park close and park far
            if (driver1.isButtonJustPressed(Button.A))
                parkClose = !parkClose;

            // If the driver presses start, we exit setup.
            if (driver1.isButtonJustPressed(Button.START))
                settingUp = false;

            // Display the current setup
            telemetry.addData("Is on red alliance: ", isRedAlliance);
            telemetry.addData("Match delay: ", delayCount);
            telemetry.addData("Is scoring foundation: ", scoreFoundation);
            telemetry.addData("Number of SkyStones: ", numSkyStones);
            telemetry.addData("Is parking close: ", parkClose);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }

        // Adjust constants based on red / blue alliance selection
        if (!isRedAlliance)
        {
            initAngle_side = -90;
            turnShift = 180;
            robotShiftSign = -1.0;
            centerAdjustment = 2.0 * stoneShift;
        }

        telemetry.log().clear();
        telemetry.log().add("Setup finished.");
    }


    // Autonomous scoring system method
//    public void runLiftAuto(int targetPos)
//    {
//        int curLiftPos = liftMotor.getCurrentPosition();
//
//        // Set target position and power
//        liftMotor.setTargetPosition(targetPos);
//        liftMotor.setPower(Constants.LIFT_POWER_FACTOR);
//
//        // While outside of tolerance, continue drive liftMotor and parallelServo
//        while (Math.abs(targetPos - curLiftPos) > Constants.LIFT_MOTOR_TOLERANCE_ENC_TICKS)
//        {
//            curLiftPos = liftMotor.getCurrentPosition();
//
//            // This yields the fraction of 1 rotation that the motor has progressed through (in other
//            // words, the range 0 - 1 corresponds to 0 - 360 degrees).
//            double deltaMotorPos = liftMotor.getCurrentPosition() / Constants.LIFT_MOTOR_TICKS;
//            // Power the servo such that it remains parallel to the ground.
//            parallelServo.setPosition(Constants.PARALLEL_SERVO_INIT + deltaMotorPos * Constants.MOTOR_TO_REV_SERVO_MOVEMENT);
//
//            // Display telemetry data
//            telemetry.addData("Parallel servo position: ", parallelServo.getPosition());
//            telemetry.addData("Grabber servo position: ", grabberServo.getPosition());
//            telemetry.addData("Lift motor position: ", liftMotor.getCurrentPosition());
//            telemetry.update();
//            idle();
//        }
//    }

    public void alignWithSkyStone() throws InterruptedException
    {
        // Left, middle, and right SkyStone yellow means
        double m1 = skystoneDetector.getMean1();
        double m2 = skystoneDetector.getMean2();
        double m3 = skystoneDetector.getMean3();

        /*
         *  If the yellow filter value of the right stone is lower than that of the middle or
         *  left stones, then it has more black and is therefore the SkyStone.
         *
         *  Otherwise, if the yellow value of the middle stone is less than that of the left
         *  stone, the middle stone is the SkyStone.
         *
         *  Failing the first two options, the left stone is the SkyStone.
         */
        if(m1 < m2 && m1 < m3)
        {
            robotShift = -stoneShift;
        }
        else if(m2 < m3)
        {
            robotShift = 0;
        }
        else
        {
            robotShift = stoneShift;
        }

        // todo Need position edits
        // Translate robot
        navigateUsingEncoders(robotShift, 0, 0.3, false);
    }


    // todo Test method again now that IMU is being used to calculate angle.
    // todo Consolidate use of floats and doubles
    /**
     * This method will drive the robot from its current position on the field to the coordinates (x, y)
     * on the field and orient the robot to face the direction w.  It uses Vuforia to determine x and y
     * coordinates; if an image target is not available, however, it updates its last known Vuforia position
     * using encoders.  Finally, it uses the IMU to determine angle.
     *
     * This method does not take into account placement of obstacles (e.g.: the Skybridge supports); thus,
     * <b><u>DO NOT CALL THIS METHOD INDISCRIMINATELY!</u></b>
     *
     * @param x           is the x-coordinate of the point the robot should move towards.
     * @param y           is the y-coordinate of the point the robot should move towards.
     * @param targetAngle is the desired orientation of the robot, in degrees.
     */
    public void driveToCoordinates(float x, float y, double targetAngle, double maxDrivePower)
    {
        // Starts true to ensure first if() statement below runs once after target is lost.
        boolean justLostTargets = true;

        // Robot navigation parameters.
        double driveAngle;
        double drivePower;
        double rotationPower;

        // Extract initial location data.
        vuf.getLocation();
        float xPos = vuf.translation.get(0) / Constants.MM_PER_INCH;   // Convert Vuforia mm to inches
        float yPos = vuf.translation.get(1) / Constants.MM_PER_INCH;
        float lastX = xPos;    // Enooder backup nav distances.
        float lastY = yPos;
        // Use IMU for angle, as Vuforia angle has odd conventions and can cause target to be lost easily if one uses it to turn.
        currentAngle = getAngularOrientationWithOffset();

        // Calculate distance from robot to target.
        float distance = (float) calculateDistance(x - xPos, y - yPos);
        // Calculate angle between robot and target.
        double angleDiff = normalizeRotationTarget(targetAngle, currentAngle);


        // Update location-----------------------------------------------------------------------------------
        // While we are outside of tolerance, continue to navigate.
        while ((distance > Constants.POSITION_TOLERANCE_IN || angleDiff > Constants.ANGLE_TOLERANCE_DEG) && !isStopRequested())
        {
            // Update location data every loop.
            vuf.getLocation();

            // We just lost target visibility and need to store last available Vuforia data.
            if (!vuf.getTargetVisibility() && justLostTargets)
            {
                // Store last acquired x and y values in case we lose all targets.
                lastX = xPos;
                lastY = yPos;

                justLostTargets = false;

                // Reset motor encoders and return them to RUN_USING_ENCODERS.
                motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // Approximate x and y position using encoders.
            else if (!vuf.getTargetVisibility() && !justLostTargets)
            {
                // todo Sign flips correct for new front of robot?
                // Update positions using last Vuforia pos + distance measured by encoders (utilizes fact that encoders have been reset to 0).
                xPos = lastX + (float) (Constants.IN_PER_ANDYMARK_TICK * (-motorFL.getCurrentPosition() +
                        motorBL.getCurrentPosition() - motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / (4 /*Math.sqrt(2)*/));
                yPos = lastY + (float) (Constants.IN_PER_ANDYMARK_TICK * (-motorFL.getCurrentPosition() -
                        motorBL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / 4);
            }
            // Update global position and angle coordinates.
            else
            {
                xPos = vuf.translation.get(0) / Constants.MM_PER_INCH;
                yPos = vuf.translation.get(1) / Constants.MM_PER_INCH;
            }

            // Get global orientation using IMU.
            currentAngle = getAngularOrientationWithOffset();
            distance = (float) calculateDistance(x - xPos, y - yPos);
            angleDiff = normalizeRotationTarget(targetAngle, currentAngle);
            //-----------------------------------------------------------------------------------------------


            // Move robot according to distance and angle data-----------------------------------------------
            // Filter and roll location data to get robot navigation parameters.
            translationFilter.roll(distance);
            drivePower = translationFilter.getFilteredValue();          // + 180 necessary to account for webcam
            driveAngle = normalizeAngle(Math.toDegrees(Math.atan2((y - yPos), (x - xPos))) + Constants.WEBCAM_1_OFFSET);  // Take atan of y / x accounting for quadrant, then convert to degrees
            // as required for autonomousDriveMecanum method.
            // todo Properly account for - sign on rotationPower.
            rotationFilter.roll(-angleDiff);
            rotationPower = rotationFilter.getFilteredValue();

            // Ensure drivePower and rotationPower are not too large.
            drivePower = Range.clip(drivePower, -maxDrivePower, maxDrivePower);
            rotationPower = Range.clip(rotationPower, -Constants.MAX_NAV_ROT_POWER, Constants.MAX_NAV_ROT_POWER);
            driveMecanum(driveAngle, drivePower, rotationPower);  // Don't want to give full rotation power.
            //-----------------------------------------------------------------------------------------------


            telemetry.addData("xPos: ", xPos);
            telemetry.addData("yPos: ", yPos);
            telemetry.addData("currentAngle:", currentAngle);
            telemetry.addData("angleDiff:", angleDiff);
            telemetry.addData("driveAngle: ", driveAngle);
            telemetry.addData("drivePower: ", drivePower);
            telemetry.addData("rotationPower: ", rotationPower);
            telemetry.update();
            idle();
        }

        stopDriveMotors();
    }

    // todo We shouldn't need these now that we have implemented PID loops

    /**
     * Note: This method is private; it should never need to be called outside of this class.
     * distancePower limits the maximum value of power to Constants.AUTONOMOUS_SCALE_DISTANCE,
     * and scales in proportion with Constants.AUTONOMOUS_SCALE_POWER when power is smaller.
     */
    private float distancePower(float power)
    {
        if (power > Constants.AUTONOMOUS_SCALE_DISTANCE)
        {
            power = Constants.AUTONOMOUS_SCALE_DISTANCE;
        } else
        {
            power /= Constants.AUTONOMOUS_SCALE_DISTANCE;
        }
        return power;
    }

    /**
     * Note: This method is private; it should never need to be called outside of this class.
     * rotationPower limits the maximum value of angle to Constants.AUTONOMOUS_SCALE_ANGLE,
     * and scales in proportion with Constants.AUTONOMOUS_SCALE_ANGLE when angle is smaller.
     */
    private float rotationPower(float angle)
    {
        if (Math.abs(angle) > Constants.AUTONOMOUS_SCALE_ANGLE)
        {
            angle = Constants.AUTONOMOUS_SCALE_ANGLE;
        } else
        {
            angle /= Constants.AUTONOMOUS_SCALE_ANGLE;
        }
        return angle;
    }
}
