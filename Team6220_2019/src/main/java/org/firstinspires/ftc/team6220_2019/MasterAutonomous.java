package org.firstinspires.ftc.team6220_2019;

/**
 * This class includes methods and fields that are useful for generic autonomous OpModes.
 */
abstract public class MasterAutonomous extends MasterOpMode
{
    // This method can be used with the skystone navigation target.  It allows a robot to navigate
    // to the front of the target and follow it if it moves.
    public void vuforiaFollowObject()
    {
        vRes.getLocation();
        float x = vRes.translation.get(0);
        float y = vRes.translation.get(1);
        float z = vRes.translation.get(2);

        float w = vRes.rotation.thirdAngle;

        float distance = distancePower((float) Math.sqrt(x * x + y * y));
        float angle = (float) (Math.atan(y / x) * 180 / Math.PI);
        if (x > 0)
        {
            angle += 180;
        }
        angle = (float) normalizeAngle(angle);

        driveMecanum(angle, distance, anglePower(-w));
    }

    public void driveToCenterLine()
    {
        vRes.getLocation();
        float x = vRes.translation.get(0);
        float y = vRes.translation.get(1);
        float z = vRes.translation.get(2);

        if (x > 24)
        {
            driveMecanum(180 - vRes.rotation.thirdAngle, Constants.AUTONOMOUS_SCALE_DISTANCE, 0);
        } else
        {
            float yCoordinate = 60;
            if(isRed){
                yCoordinate *= -1;
            }
            float distance = distancePower((float) Math.sqrt(x * x + (y - yCoordinate) * (y - yCoordinate)));

            // WORK HERE
            float angle = (float) (Math.atan(y / x) * 180 / Math.PI);
            if (x > 0)
            {
                angle += 180;
            }
            angle = (float) normalizeAngle(angle);
        }
    }

    /**
     * distancePower limits the maximum value of power to Constants.AUTONOMOUS_SCALE_DISTANCE,
     * and scales in proportion with Constants.AUTONOMOUS_SCALE_POWER when power is smaller.
     */
    public float distancePower(float power)
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
     * anglePower limits the maximum value of angle to Constants.AUTONOMOUS_SCALE_ANGLE,
     * and scales in proportion with Constants.AUTONOMOUS_SCALE_ANGLE when angle is smaller.
     */
    public float anglePower(float angle)
    {
        if (angle > Constants.AUTONOMOUS_SCALE_ANGLE)
        {
            angle = Constants.AUTONOMOUS_SCALE_ANGLE;
        } else
        {
            angle /= Constants.AUTONOMOUS_SCALE_ANGLE;
        }
        return angle;
    }
}
