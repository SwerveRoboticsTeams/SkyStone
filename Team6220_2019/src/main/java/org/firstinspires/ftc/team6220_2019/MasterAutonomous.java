package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;

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
    // Whether we want to score the foundation or not
    boolean scoreFoundation = true;
    // Whether we want to park on far or close end of line under alliance Skybridge
    boolean parkClose = true;
    //------------------------------------------------------------------------------------------

    // Initialize angles and distances for various differing setup options----------------------
    double stoneShift = 8; // Centers of stones are 8 in apart
    double centerOffset = 6.5;
    double robotShift = centerOffset;     // Actual distance the robot moved (will be changed)

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
    // Also needs to pass in Vuforia use boolean
    @Override
    public void initialize(boolean isUsingVuforia)
    {
        super.initialize(isUsingVuforia);
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
            centerAdjustment = 2.0 * centerOffset;
        }

        telemetry.log().clear();
        telemetry.log().add("Setup finished.");
    }


    // Autonomous scoring system method
    public void runScoringSystemAuto(int targetPos)
    {
        int curLiftPos = liftMotor.getCurrentPosition();

        // Set target position and power
        liftMotor.setTargetPosition(targetPos);
        liftMotor.setPower(Constants.LIFT_POWER_FACTOR);

        // While outside of tolerance, continue drive liftMotor and parallelServo
        while (Math.abs(targetPos - curLiftPos) > Constants.LIFT_MOTOR_TOLERANCE_ENC_TICKS)
        {
            curLiftPos = liftMotor.getCurrentPosition();

            // This yields the fraction of 1 rotation that the motor has progressed through (in other
            // words, the range 0 - 1 corresponds to 0 - 360 degrees).
            double deltaMotorPos = liftMotor.getCurrentPosition() / Constants.LIFT_MOTOR_TICKS;
            // Power the servo such that it remains parallel to the ground.
            parallelServo.setPosition(Constants.PARALLEL_SERVO_INIT + deltaMotorPos * Constants.MOTOR_TO_REV_SERVO_MOVEMENT);

            // Display telemetry data
            telemetry.addData("Parallel servo position: ", parallelServo.getPosition());
            telemetry.addData("Grabber servo position: ", grabberServo.getPosition());
            telemetry.addData("Lift motor position: ", liftMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }


    /**
     * UNTESTED - - - UNTESTED - - - UNTESTED
     * This method identifies the location of the SkyStone, using this information to determine
     * whether it is in the left, center, or right orientation.  Next, it uses navigateUsingEncoders()
     * to align the robot head-on with the SkyStone.
     */
    public void vuforiaAlignWithSkyStone()
    {
        vRes.getLocation();

        if (!vRes.getTargetVisibility())
        {
            // Shift toward center stone if we can't ID target
        } else
        {
            // Align with SkyStone
            vRes.getLocation();
            // todo This should use vRes.CAMERA_LEFT_DISPLACEMENT (in inches, not MM)
            double stoneDistance = vRes.translation.get(1) / Constants.MM_PER_INCH;
            telemetry.addData("stoneDistance: ", stoneDistance);
            telemetry.update();

            if (stoneDistance > (stoneShift - 2))  // Need to shift right
                robotShift = stoneShift + 6.5;
            else if (stoneDistance < (-stoneShift + 2))    // Need to shift left
                robotShift = -stoneShift + 5.5; // todo Adjust further if necessary
            // Otherwise, no change in shift is necessary (robotShift = 6.5 by default)
        }

        // Translate robot
        navigateUsingEncoders(robotShift, 0, 0.3, false);
    }


    // todo Test and adjust method
    // This method can be used with the skystone navigation target.  It allows a robot to navigate
    // to the front of the target and follow it if it moves.

    /**
     * UNTESTED - - - UNTESTED - - - UNTESTED
     * NOTE: DO NOT USE THIS METHOD IN COMPETITION. THIS IS A TEST METHOD ONLY.
     * With a skystone defined as (0, 0, 0) absolutely, this method is designed to make the robot drive towards
     * the skystone at all times. It should follow a skystone if moved.
     */
    public void vuforiaFollowObject()
    {
        // Updates rotation and translation vectors
        vRes.getLocation();


        double distanceToStone = 100;

        if (!vRes.getTargetVisibility())
        {
            driveMecanum(0, 0, 0.10);
        } else
        {
            if (distanceToStone > Constants.POSITION_TOLERANCE_IN)
            {
                // x, y, z defines the location of the center of the robot relative to the skystone (in inches).
                // todo Need to test whether distance - 8 works correctly.
                double x = vRes.translation.get(0) / Constants.MM_PER_INCH + 8;
                double y = vRes.translation.get(1) / Constants.MM_PER_INCH;
                double z = vRes.translation.get(2) / Constants.MM_PER_INCH;
                telemetry.addData("x value: ", x);
                telemetry.addData("y value: ", y);
                telemetry.addData("z value: ", z);
                // w is the rotation of the robot relative to the skystone in degrees.
                double currentAngle = vRes.rotation.firstAngle;

                distanceToStone = calculateDistance(x, y);
                // todo - signs should be properly accounted for.
                // Transform position and heading diffs to linear and rotation powers using filters----
                translationFilter.roll(distanceToStone);
                double drivePower = translationFilter.getFilteredValue();
                rotationFilter.roll(currentAngle);
                double rotationPower = rotationFilter.getFilteredValue();

                double angle = (Math.atan(y / x) * 180 / Math.PI);
                if (x > 0)
                {
                    angle += 180;
                }
                angle = normalizeAngle(angle);

                // Drive robot.  Need to add 90 degrees to account for phone orientation relative to robot.
                autonomousDriveMecanum(angle + 90, drivePower * 0.05, 0/*rotationPower*/);
            } else
                stopDriveMotors();
        }
    }

    // todo Implement (global coordinates)

    /**
     * UNTESTED - - - UNTESTED - - - UNTESTED
     * DO NOT CALL THIS METHOD TO DRIVE TO THE BRIDGE IN THE MIDDLE OF AUTONOMOUS. DESPITE THE
     * SIMILARITY WITH driveToBridge, THIS METHOD DOES NOT MAKE THE ROBOT SLOW DOWN WHEN
     * APPROACHING THE BRIDGE.
     * This method is designed to be called at the end of autonomous to drive the robot to the
     * center line. It consists of three stages.
     * 1. If the robot is far enough in the +x direction (near the foundations or buildzone), the
     * robot will translate in the -x direction to avoid any chance of collision with the
     * foundation.
     * 2. If the robot is to the left or right of the middle bridge, translate up or down to the
     * team bridge.
     * 3. Finally, drive to a predetermined point under the team bridge.
     */
    public void driveCenterLineEndAutonomous()
    {
        vRes.getLocation();
        // (x, y, z) is the location of the robot on the field.
        float x = vRes.translation.get(0);
        float y = vRes.translation.get(1);
        float z = vRes.translation.get(2);

        // Conditional to check for step one. 24 is an arbitrary number that may need adjusting.
        if (x > 24)
        {
            autonomousDriveMecanum(180 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
        }
        // Conditional to check for step two. 36 is a partially arbitrary number that may need adjusting.
        else if (Math.abs(y) < 36)
        {
            // Drive up if we are the blue team, and drive down if we are the red team.
            if (isRedAlliance)
            {
                autonomousDriveMecanum(-90 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
            } else
            {
                autonomousDriveMecanum(90 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
            }
        }
        // If neither of the first two steps are true, then the third step is executed.
        else
        {
            // This variable controls the y-coordinate of where we want our robot to park.
            float yCoordinate = 48; //MUST BE POSITIVE!!! THE ROBOT WILL PARK ON THE WRONG SIDE IF NEGATIVE!!!
            // Flip if we are the red team.
            if (isRedAlliance)
            {
                yCoordinate *= -1;
            }
            float distance = distancePower((float) Math.sqrt(x * x + (y - yCoordinate) * (y - yCoordinate)));
            float angle = (float) (Math.atan(y / x) * 180 / Math.PI);

            if (x > 0)
            {
                angle += 180;
            }
            angle = (float) normalizeAngle(angle);

            autonomousDriveMecanum(angle - vRes.rotation.firstAngle, distancePower(distance), 0);
        }
    }

    // todo Implement (global coordinates)

    /**
     * UNTESTED - - - UNTESTED - - - UNTESTED
     * DO NOT USE THIS METHOD AT THE END OF AUTONOMOUS. DESPITE THE SIMILARITY TO
     * driveCenterLineEndAutonomous, IT IS NOT DESIGNED TO MAKE THE ROBOT PARK NICELY.
     * This method is designed to be called during autonomous to drive the robot through the
     * center bridges in either direction. It consists of three stages.
     * 1. If the robot is far enough in the +x direction (near the foundations or buildzone), the
     * robot will translate in the -x direction to avoid any chance of collision with the
     * foundation. This step will be skipped if the robot is on the depot side.
     * 2. Translate up or down as necessary to align with the desired bridge.
     * 3. Finally, drive through the bridge.
     * Note: This method is only intended to drive to the bridge. This method will not drive the
     * robot away from the bridge; use a different method for that purpose.
     *
     * @param bridge is positive if we want to drive through our team bridge. Otherwise (zero or
     *               negative, we want to drive through the center bridge.
     */
    public void driveToBridge(int bridge)
    {
        vRes.getLocation();
        // (x, y, z) is the location of the robot on the field.
        float x = vRes.translation.get(0);
        float y = vRes.translation.get(1);
        float z = vRes.translation.get(2);

        int foundationTolerance = 24;
        int teamBridgeTolerance = 36;
        int centerTolerance = 24;

        // Conditional to check for step one. foundationTolerance is an arbitrary number that may need adjusting.
        if (x > foundationTolerance)
        {
            autonomousDriveMecanum(180 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
        }
        // Conditional to check for step two. teamBridgeTolerance and centerTolerance are partially arbitrary
        // numbers that may need adjusting.
        else if ((Math.abs(y) < teamBridgeTolerance && bridge > 0) || (Math.abs(y) > centerTolerance && bridge <= 0))
        {
            // Check if we want to go to our team bridge.
            if (bridge > 0)
            {
                // Drive up if we are the blue team, and drive down if we are the red team.
                if (isRedAlliance)
                {
                    autonomousDriveMecanum(-90 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
                } else
                {
                    autonomousDriveMecanum(90 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
                }
            }
            // Otherwise, go to center bridge.
            else
            {
                if (y > centerTolerance)
                {
                    autonomousDriveMecanum(-90 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
                } else if (y < -centerTolerance)
                {
                    autonomousDriveMecanum(90 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
                }
            }
        }
        // If neither of the first two steps are true, then the third step is executed.
        else
        {
            // If we are left of the bridge, drive right.
            if (x < 0)
            {
                autonomousDriveMecanum(-vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
            }
            // Else, drive left.
            else
            {
                autonomousDriveMecanum(180 - vRes.rotation.firstAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
            }
        }
    }

    /**
     * UNTESTED - - - UNTESTED - - - UNTESTED
     * This method will drive the robot from its current position on the field to the coordinates (x, y) on the field, and orient the robot
     * to face the direction w. It does not take into account placement of obstacles (e.g.: the Skybridge supports); thus,
     * <b><u>DO NOT CALL THIS METHOD INDISCRIMINATELY!</u></b>
     *
     * @param x is the x-coordinate of the point the robot should move towards.
     * @param y is the y-coordinate of the point the robot should move towards.
     * @param w is the desired orientation of the robot, in degrees.
     */
    public boolean driveToCoordinates(double x, double y, double w)
    {
        vRes.getLocation();

        float xPos = vRes.translation.get(0);
        float yPos = vRes.translation.get(1);
        float wRot = vRes.rotation.firstAngle;

        telemetry.addData("xPos: ", xPos);
        telemetry.addData("yPos: ", yPos);
        telemetry.update();

        double driveAngle = Math.atan((y - yPos) / (x - xPos));
        if ((x - xPos) > 0)
        {
            driveAngle += 180;
        }

        float distance = (float) Math.sqrt((y - yPos) * (y - yPos) + (x - xPos) * (x - xPos));

        double power = distancePower(distance);

        if (Math.abs(power) > Constants.MAX_DRIVE_POWER)
            autonomousDriveMecanum(driveAngle, Constants.MAX_DRIVE_POWER, 0/*rotationPower((float)w - wRot)*/);
        else
            autonomousDriveMecanum(driveAngle, distancePower(distance), 0/*rotationPower((float)w - wRot)*/);

        if (distance < Constants.POSITION_TOLERANCE_IN)
        { // functions as a tolerance variable.
            return true;
        } else
        {
            return false;
        }
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
