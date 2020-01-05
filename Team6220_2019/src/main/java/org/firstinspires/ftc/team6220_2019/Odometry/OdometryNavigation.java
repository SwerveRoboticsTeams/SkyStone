package org.firstinspires.ftc.team6220_2019.Odometry;

import org.firstinspires.ftc.team6220_2019.MasterOpMode;

/**
 * This class includes important constants and methods for odometry navigation (currently in progress).
 */

public abstract class OdometryNavigation extends MasterOpMode
{
    public void driveToPosition(double x, double y, double movementSpeed)
    {
        double globalXPos = 0;
        double globalYPos = 0;
        double globalAngle = 0;

        // Need to get distance and angle between robot and target.
        double distanceToTarget = Math.hypot(x-globalXPos, y-globalYPos);
        double absoluteAngleToTarget = Math.atan2(y - globalYPos, x - globalXPos);
        double relativeAngleToTarget = normalizeAngle(absoluteAngleToTarget - getAngularOrientationWithOffset());

        // Calculate relative x and y distances to our target.
        double relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

        // Calculate movement x and y powers, which must be normalized to prevent excessively large
        // or small power values.
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        // todo Where are movement_x and movement_y supposed to be initialized?  Also, how does this integrate with simulator?
        // todo Also, size of field = 140.94 in, not 144 in.  (Should probably adjust for this value...)
        /*movement_x = movementXPower;
        movement_y = movementXPower;*/
    }
}
