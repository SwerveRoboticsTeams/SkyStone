package org.firstinspires.ftc.team6220_2019;

public abstract class OdometryNavigation extends MasterOpMode
{
    public void driveToPosition(double x, double y, double movementSpeed)
    {
        double globalXPos = 0;
        double globalYPos = 0;
        double globalAngle = 0;

        // Need to get both absolute and relative orientations of robot.
        double absoluteAngleToTarget = Math.atan2(y - globalYPos, x - globalXPos);
        double relativeAngleToTarget = normalizeAngle(absoluteAngleToTarget - getAngularOrientationWithOffset());
    }
}
