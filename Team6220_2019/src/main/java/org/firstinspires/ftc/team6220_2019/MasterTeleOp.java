package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class MasterTeleOp extends MasterOpMode
{
    // Start robot in slow mode (per Slater)
    boolean slowMode = false;
    // Allows us to switch front of robot.
    boolean driveReversed = true;

    // The following booleans help the program keep track of which step of placing a stone the grabber is in.
    boolean hasLoweredArm = false;
    boolean hasGrabbedStone = false;
    boolean hasRotatedArm = false;
    boolean hasPlacedStone = false;

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
            } else
            {
                tFactor = Constants.T_FACTOR;
                rFactor = Constants.R_FACTOR;
            }

            slowMode = !slowMode;
        }


        // Stick inputs must be changed from x and y to angle, drive power, and rotation power---
        double angle = Math.toDegrees(driver1.getRightStickAngle());

        double drivePower = tFactor * stickCurve.getOuput(driver1.getRightStickMagnitude());

        double rotationPower = rFactor * stickCurve.getOuput(driver1.getLeftStickX());
        //----------------------------------------------------------------------------------------


        // Change drive direction if Driver 1 presses left bumper
        if (driver1.isButtonJustPressed(Button.LEFT_BUMPER) && !driveReversed)
            driveReversed = true;
        else if (driver1.isButtonJustPressed(Button.LEFT_BUMPER) && driveReversed)
            driveReversed = false;

        // Drive in direction based on whether driveDirectionShift is true
        if (!driveReversed)
            driveMecanum(angle, drivePower, rotationPower);
        else
            driveMecanum(angle + 180, drivePower, rotationPower);

        // Add telemetry data for drivers
        telemetry.addData("angle: ", angle);
        telemetry.addData("drivePower: ", drivePower);
        telemetry.addData("rotationPower: ", rotationPower);
        telemetry.addData("Left stick x: ", driver1.getLeftStickX());
        telemetry.addData("Right stick x: ", driver1.getRightStickX());
    }


    // todo Make sure this works
    // TeleOp method for driving collector.
    public void driveCollector()
    {
        // Drive linear slide-----------------------------------------------------------------------
        double leftTrigger = driver1.getLeftTriggerValue(), rightTrigger = driver1.getRightTriggerValue();

        if(rightTrigger >= leftTrigger && rightTrigger > Constants.MINIMUM_TRIGGER_VALUE) // rightTrigger extends
        {
            runSlideMotor(rightTrigger, Constants.SLIDE_MOTOR_MAX_POWER);
        }
        else if(leftTrigger > rightTrigger && leftTrigger > 0) //leftTrigger retracts
        {
            runSlideMotor(-1 * leftTrigger, Constants.SLIDE_MOTOR_MAX_POWER);
        }
        else
        {
            runSlideMotor(0, Constants.SLIDE_MOTOR_MAX_POWER);
        }
        //------------------------------------------------------------------------------------------

        // Drive collector motors-------------------------------------------------------------------
        if (driver1.isButtonPressed(Button.B))    // Spit out stone
            runCollector(false, false);
        else if (driver1.isButtonPressed(Button.A))     // Collect stone
            runCollector(true, false);
        else    // Make sure that if neither DPAD_UP or DPAD_DOWN are pressed, the motors don't continue running
        {
            collectorLeft.setPower(0);
            collectorRight.setPower(0);
        }
        //------------------------------------------------------------------------------------------
    }


    /*// todo Needs to be adjusted from arm to linear slide code.
    // TeleOp scoring system method.  Uses liftMotor to move scoring arm, with parallelServo
    // keeping grabber parallel to the ground.
    public void driveLift()
    {
        //double leftTrigger = driver1.getLeftTriggerValue(), rightTrigger = driver1.getRightTriggerValue();
        double rightStickY = driver2.getRightStickY();

        // Linear slides / raising mechanism should be idle if neither or both triggers are pressed
        // Change drive mode if stick is pressed a significant amount and we were in RUN_TO_POSITION
        if (Math.abs(rightStickY) >= Constants.MINIMUM_JOYSTICK_POWER && isRunToPosMode)
        {
            isRunToPosMode = false;
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            hasLoweredArm = false;
            hasGrabbedStone = false;
            hasRotatedArm = false;
            hasPlacedStone = false;
        }

        if (!isRunToPosMode)
            liftMotor.setPower(-rightStickY * Constants.LIFT_POWER_FACTOR);

        // This yields the fraction of 1 rotation that the motor has progressed through (in other
        // words, the range 0 - 1 corresponds to 0 - 360 degrees).
        double deltaMotorPos = liftMotor.getCurrentPosition() / Constants.LIFT_MOTOR_TICKS;
        // Power the servo such that it remains parallel to the ground.
        parallelServo.setPosition(Constants.PARALLEL_SERVO_INIT + deltaMotorPos * 1.55*//*2*//*);   // todo Does 2 need to be 4/3?


        // Code for automatic movement of arm-------------------------------------------------------------------
        // If driver 2 presses A, return lift to position just high enough to grab stone
        if (driver2.isButtonJustPressed(Button.A))
        {
            isRunToPosMode = true;
            liftMotor.setTargetPosition(Constants.LIFT_GRAB_POS);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(Constants.LIFT_POWER_FACTOR);

            grabberServo.setPosition(Constants.GRABBER_OPEN);
        }

        // todo Implement once we have time
        if (isRunToPosMode)
        {
            if (!hasLoweredArm)
            {
                if (Math.abs(liftMotor.getTargetPosition() - Constants.LIFT_GRAB_POS) <= Constants.LIFT_MOTOR_TOLERANCE_ENC_TICKS)
                {
                    hasLoweredArm = true;
                } else
                {
                    liftMotor.setTargetPosition(Constants.LIFT_GRAB_POS);
                }
            } else if (!hasGrabbedStone)
            {
                grabberServo.setPosition(Constants.GRABBER_CLOSED);
                hasGrabbedStone = true;
            } else if (!hasRotatedArm)
            {
                if (Math.abs(liftMotor.getTargetPosition() - Constants.LIFT_PLACE_POS) <= Constants.LIFT_MOTOR_TOLERANCE_ENC_TICKS)
                {
                    hasRotatedArm = true;
                } else
                {
                    liftMotor.setTargetPosition(Constants.LIFT_PLACE_POS - Constants.NUM_TICKS_PER_STONE * towerHeight);
                }
            } else if (!hasPlacedStone)
            {
                grabberServo.setPosition(Constants.GRABBER_OPEN);
                hasPlacedStone = true;
            } else
            {
                towerHeight++;
                isRunToPosMode = false;
            }
        }


        // Display telemetry data to drivers
        telemetry.addData("Driver 2 right stick y: ", rightStickY);
        telemetry.addData("Parallel servo position: ", parallelServo.getPosition());
        telemetry.addData("Grabber servo position: ", grabberServo.getPosition());
        telemetry.addData("Lift motor position: ", liftMotor.getCurrentPosition());
    }*/
}