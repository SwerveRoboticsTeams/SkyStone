package org.firstinspires.ftc.team6220_2019;

/**
 * This class includes methods and fields that are useful for generic autonomous OpModes.
 */
abstract public class MasterAutonomous extends MasterOpMode
{
    @Override
    public void initialize()
    {
        super.initialize();
        vRes.initVuforia();
    }

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
        vRes.getLocation();

        if (!vRes.getTargetVisibility())
        {
            driveMecanum(0, 0, 0.10);
        }
        else
        {
            // x, y, z defines the location of the center of the robot relative to the skystone.
            // todo Need to test whether distance - 8 works correctly.
            float x = vRes.translation.get(0) / Constants.MM_PER_INCH - 8;
            float y = vRes.translation.get(1) / Constants.MM_PER_INCH;
            float z = vRes.translation.get(2) / Constants.MM_PER_INCH;
            telemetry.addData("x value: ", x);
            telemetry.addData("y value: ", y);
            telemetry.addData("z value: ", z);
            // w is the rotation of the robot relative to the skystone in degrees.
            float w = vRes.rotation.thirdAngle;

            float distance = distancePower((float) Math.sqrt(x * x + y * y));

            float angle = (float) (Math.atan(y / x) * 180 / Math.PI);
            if (x > 0)
            {
                angle += 180;
            }
            angle = (float) normalizeAngle(angle);

            driveMecanum(angle, distance, rotationPower(w));
            //driveMecanum(0, 0, 0);
        }
    }

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
            driveMecanum(180 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
        }
        // Conditional to check for step two. 36 is a partially arbitrary number that may need adjusting.
        else if (Math.abs(y) < 36)
        {
            // Drive up if we are the blue team, and drive down if we are the red team.
            if (isRed)
            {
                driveMecanum(-90 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
            } else
            {
                driveMecanum(90 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
            }
        }
        // If neither of the first two steps are true, then the third step is executed.
        else
        {
            // This variable controls the y-coordinate of where we want our robot to park.
            float yCoordinate = 48; //MUST BE POSITIVE!!! THE ROBOT WILL PARK ON THE WRONG SIDE IF NEGATIVE!!!
            // Flip if we are the red team.
            if (isRed)
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

            driveMecanum(angle - vRes.rotation.thirdAngle, distancePower(distance), 0);
        }
    }

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
            driveMecanum(180 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
        }
        // Conditional to check for step two. teamBridgeTolerance and centerTolerance are partially arbitrary
        // numbers that may need adjusting.
        else if ((Math.abs(y) < teamBridgeTolerance && bridge > 0) || (Math.abs(y) > centerTolerance && bridge <= 0))
        {
            // Check if we want to go to our team bridge.
            if (bridge > 0)
            {
                // Drive up if we are the blue team, and drive down if we are the red team.
                if (isRed)
                {
                    driveMecanum(-90 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
                } else
                {
                    driveMecanum(90 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
                }
            }
            // Otherwise, go to center bridge.
            else
            {
                if (y > centerTolerance)
                {
                    driveMecanum(-90 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
                } else if (y < -centerTolerance)
                {
                    driveMecanum(90 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
                }
            }
        }
        // If neither of the first two steps are true, then the third step is executed.
        else
        {
            // If we are left of the bridge, drive right.
            if (x < 0)
            {
                driveMecanum(-vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
            }
            // Else, drive left.
            else
            {
                driveMecanum(180 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
            }
        }
    }

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
