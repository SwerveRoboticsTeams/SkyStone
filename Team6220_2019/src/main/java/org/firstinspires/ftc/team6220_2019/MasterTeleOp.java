package org.firstinspires.ftc.team6220_2019;

abstract public class MasterTeleOp extends MasterOpMode
{
    // Start robot in slow mode (per Slater)
    boolean slowMode = false;
    // Allows us to switch front of robot.
    boolean driveReversed = true;
    // Remembers whether the grabber is open or closed.
    boolean isGrabberOpen = true;

    // Factor that adjusts magnitudes of vertical and horizontal movement.
    double tFactor = Constants.T_FACTOR;
    // Factor that adjusts magnitude of rotational movement.
    double rFactor = Constants.R_FACTOR;


    public void driveMecanumWithJoysticks()
    {
        // Note: factors are different for translation and rotation
        // Slow mode functionality.  1st driver presses right bumper to toggle slow mode
        if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER))
        {
            if (!slowMode)
            {
                tFactor = Constants.SLOW_MODE_T_FACTOR;
                rFactor = Constants.SLOW_MODE_R_FACTOR;
            }
            else
            {
                tFactor = Constants.T_FACTOR;
                rFactor = Constants.R_FACTOR;
            }

            slowMode = !slowMode;
        }


        // Stick inputs must be changed from x and y to angle, drive power, and rotation power---
        double angle = Math.toDegrees(driver1.getRightStickAngle());

        double drivePower = tFactor * stickCurve.getOuput(driver1.getRightStickMagnitude());

        double rotationPower = rFactor * stickCurve.getOuput(gamepad1.left_stick_x);
        //----------------------------------------------------------------------------------------


        // Change drive direction based on driver input
        if (driver1.isButtonJustPressed(Button.LEFT_BUMPER) && !driveReversed)
            driveReversed = true;
        else if (driver1.isButtonJustPressed(Button.LEFT_BUMPER) && driveReversed)
            driveReversed = false;

        // Drive in direction based on whether driveDirectionShift is true
        if (!driveReversed)
            driveMecanum(angle, drivePower, rotationPower);
        else
            driveMecanum(angle + 180, drivePower, rotationPower);
    }


    public void activateCollector()
    {
        // Double to control power of collector motors
        double power = 0.75;

        if(driver1.isButtonPressed(Button.DPAD_UP)){
            collectorLeft.setPower(-power);
            collectorRight.setPower(power);
        }
        else if(driver1.isButtonPressed(Button.DPAD_DOWN)){
            collectorLeft.setPower(power);
            collectorRight.setPower(-power);
        }
        // Make sure that if neither DPAD_UP or DPAD_DOWN are pressed that the motors don't continue running
        else{
            collectorLeft.setPower(0);
            collectorRight.setPower(0);
        }
    }


    // Uses liftMotor to move scoring arm, with parallelServo keeping grabber parallel to the ground.
    public void raiseScoringSystem()
    {
        double leftTrigger = driver1.getLeftTriggerValue(), rightTrigger = driver1.getRightTriggerValue();

        // Linear slides / raising mechanism should be idle if neither or both triggers are pressed
        if(leftTrigger >= Constants.MINIMUM_TRIGGER_VALUE && rightTrigger <= Constants.MINIMUM_TRIGGER_VALUE) //lowers
        {
            liftMotor.setPower(1.0 * Constants.LIFT_POWER_FACTOR_UP);
        }
        else if(rightTrigger >= Constants.MINIMUM_TRIGGER_VALUE && leftTrigger <= Constants.MINIMUM_TRIGGER_VALUE) //raises
        {
            liftMotor.setPower(-1.0 * Constants.LIFT_POWER_FACTOR_DOWN);
        }
        else
        {
            liftMotor.setPower(0);
        }

        //double deltaMotorPos = 360 * liftMotor.getCurrentPosition() / Constants.LIFT_MOTOR_TICKS;
        //double deltaServoPos = (deltaMotorPos) / 360;

        //parallelServo.setPosition(Constants.PARALLEL_SERVO_INIT + deltaServoPos);

        telemetry.addData("Parallel servo position: ", parallelServo.getPosition());
        telemetry.addData("Grabber servo position: ", grabberServo.getPosition());
        //telemetry.addData("Lift motor position: ", liftMotor.getCurrentPosition());
        //telemetry.addData("Lift motor power: ", liftMotor.getPower());
    }


    public void toggleGrabber()
    {
        // Only activate if Button.A is just pressed.
        if(driver1.isButtonJustPressed(Button.A))
        {
            // Toggle position
            if (isGrabberOpen)
            {
                grabberServo.setPosition(Constants.GRABBER_CLOSED);
            }
            else
            {
                grabberServo.setPosition(Constants.GRABBER_OPEN);
            }
            isGrabberOpen = !isGrabberOpen;
        }
    }
}