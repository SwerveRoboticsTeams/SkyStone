package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class MasterTeleOp extends MasterOpMode
{
    // Start robot in slow mode (per Slater)
    boolean slowMode = false;
    // Allows us to switch front of robot.
    boolean driveReversed = true;
    // Tells us what drive mode the lift motor is in; by default, we use RUN_USING_ENCODER
    boolean isRunToPosMode = false;
    // Remembers whether the grabber is open or closed.
    boolean isGrabberOpen = true;

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


    public void driveCollector()
    {
        if (driver2.isButtonPressed(Button.DPAD_UP))
        {
            collectorLeft.setPower(-Constants.COLLECTOR_POWER);
            collectorRight.setPower(Constants.COLLECTOR_POWER);
        }
        else if (driver2.isButtonPressed(Button.DPAD_DOWN))
        {
            collectorLeft.setPower(Constants.COLLECTOR_POWER);
            collectorRight.setPower(-Constants.COLLECTOR_POWER);
        }
        // Make sure that if neither DPAD_UP or DPAD_DOWN are pressed, the motors don't continue running
        else
        {
            collectorLeft.setPower(0);
            collectorRight.setPower(0);
        }
    }


    // Uses liftMotor to move scoring arm, with parallelServo keeping grabber parallel to the ground.
    public void driveScoringSystem()
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

        liftMotor.setPower(rightStickY * Constants.LIFT_POWER_FACTOR);

        // This yields the fraction of 1 rotation that the motor has progressed through (in other
        // words, the range 0 - 1 corresponds to 0 - 360 degrees).
        double deltaMotorPos = liftMotor.getCurrentPosition() / Constants.LIFT_MOTOR_TICKS;
        // Power the servo such that it remains parallel to the ground.
        parallelServo.setPosition(Constants.PARALLEL_SERVO_INIT + deltaMotorPos * 2);



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

        // todo Implement once encoder is working
        /*
        if(isRunToPosMode)
        {
            if(!hasLoweredArm)
            {
                if(liftMotor.getTargetPosition() == Constants.LIFT_GRAB_POS)
                {
                    hasLoweredArm = true;
                }
                else
                {
                    liftMotor.setTargetPosition(Constants.LIFT_GRAB_POS);
                }
            }
            else if(!hasGrabbedStone)
            {
                if(grabberServo.getPosition() == Constants.GRABBER_CLOSED)
                {
                    hasGrabbedStone = true;
                }
                else
                {
                    grabberServo.setPosition(Constants.GRABBER_CLOSED);
                }
            }
            else if(!hasRotatedArm)
            {
                if(liftMotor.getTargetPosition() == Constants.LIFT_PLACE_POS)
                {
                    hasRotatedArm = true;
                }
                else
                {
                    liftMotor.setTargetPosition(Constants.LIFT_PLACE_POS - Constants.NUM_TICKS_PER_STONE * towerHeight);
                }
            }
            else if(!hasPlacedStone)
            {
                if(grabberServo.getPosition() == Constants.GRABBER_OPEN)
                {
                    hasPlacedStone = true;
                }
                else
                {
                    grabberServo.setPosition(Constants.GRABBER_OPEN);
                }
            }
            else{
                towerHeight++;
                isRunToPosMode = false;
            }
        }
         */


        // Display telemetry data to drivers
        telemetry.addData("Right stick y: ", rightStickY);
        telemetry.addData("Parallel servo position: ", parallelServo.getPosition());
        telemetry.addData("Grabber servo position: ", grabberServo.getPosition());
        telemetry.addData("Lift motor position: ", liftMotor.getCurrentPosition());
    }


    public void toggleGrabber()
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