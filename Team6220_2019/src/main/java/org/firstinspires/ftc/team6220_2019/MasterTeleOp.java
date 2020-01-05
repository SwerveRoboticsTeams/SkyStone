package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2019.ResourceClasses.Button;

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
        double angle = Math.toDegrees(driver1.getLeftStickAngle());

        double drivePower = tFactor * stickCurve.getOuput(driver1.getLeftStickMagnitude());

        double rotationPower = rFactor * stickCurve.getOuput(driver1.getRightStickX());
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
        /*telemetry.addData("Left stick x: ", driver1.getLeftStickX());
        telemetry.addData("Left stick y: ", driver1.getLeftStickY());
        telemetry.addData("Right stick x: ", driver1.getRightStickX());
        telemetry.addData("Right stick y: ", driver1.getRightStickY());*/
    }


    // TeleOp method for driving collector.  Controlled by driver 2.
    public void driveCollector()
    {
        // Drive collector motors-------------------------------------------------------------------
        if (driver1.isButtonPressed(Button.DPAD_UP))    // Spit out stone
            runCollector(false, false);
        else if (driver1.isButtonPressed(Button.DPAD_DOWN))     // Collect stone
            runCollector(true, false);
        else if (driver1.isButtonPressed(Button.DPAD_LEFT))     // Rotate stone left
            runCollector(false, true);
        else if (driver1.isButtonPressed(Button.DPAD_RIGHT))     // Rotate stone right
            runCollector(true, true);
        else    // Make sure that if neither DPAD_UP or DPAD_DOWN are pressed, the motors don't continue running
        {
            collectorLeft.setPower(0);
            collectorRight.setPower(0);
        }
        //------------------------------------------------------------------------------------------
    }


    // todo Needs RUN_TO_POSITION mode and automatic distances fixed.
    // TeleOp scoring system method.  Uses liftMotor to move scoring arm, with parallelServo
    // keeping grabber parallel to the ground.
    public void driveLift()
    {
        double rightStickY = driver2.getRightStickY();

        // Change drive mode if stick is pressed a significant amount and we were in RUN_TO_POSITION
        if (Math.abs(rightStickY) >= Constants.MINIMUM_JOYSTICK_POWER && isLiftRunToPosMode)
        {
            isLiftRunToPosMode = false;
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hasLoweredArm = false;
            hasGrabbedStone = false;
            hasRotatedArm = false;
            hasPlacedStone = false;
        }

        if (!isLiftRunToPosMode)
        {
            // todo Make sure that reversing these signs was correct.
            liftMotor1.setPower(rightStickY * Constants.LIFT_POWER_FACTOR);
            liftMotor2.setPower(-rightStickY * Constants.LIFT_POWER_FACTOR);
        }

        // Code for automatic movement of arm-------------------------------------------------------------------
        // If driver 2 presses B, grabber closes.
        if (driver2.isButtonJustPressed(Button.B))
            grabberServo.setPosition(Constants.GRABBER_CLOSED);

        // If driver 2 presses Y, arm extends.
        if (driver2.isButtonJustPressed(Button.Y))
        {
            grabberArmLeft.setPosition(Constants.GRABBER_ARM_SERVO_LEFT_EXTEND);
            grabberArmRight.setPosition(Constants.GRABBER_ARM_SERVO_RIGHT_EXTEND);
        }

        // If driver 2 presses A, return lift to position just high enough to grab stone
        if (driver2.isButtonJustPressed(Button.A))
        {
            isLiftRunToPosMode = true;

            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setPower(Constants.LIFT_POWER_FACTOR);
            liftMotor1.setTargetPosition(Constants.LIFT_GRAB_POS);
            // todo Need to change this—can't use RUN_TO_POSITION on LM2 when only LM1 has an encoder.
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setPower(Constants.LIFT_POWER_FACTOR);
            liftMotor2.setTargetPosition(-Constants.LIFT_GRAB_POS);

            grabberArmLeft.setPosition(Constants.GRABBER_ARM_SERVO_LEFT_RETRACT);
            grabberArmRight.setPosition(Constants.GRABBER_ARM_SERVO_RIGHT_RETRACT);
            grabberServo.setPosition(Constants.GRABBER_OPEN);
            isGrabberOpen = true;
        }


        // todo Implement once we have time
        /*
        if (isLiftRunToPosMode)
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
                isLiftRunToPosMode = false;
            }
        }
        */


        // Display telemetry data to drivers
        telemetry.addData("Driver 2 right stick y: ", rightStickY);
        telemetry.addData("Grabber servo position: ", grabberServo.getPosition());
        telemetry.addData("Lift motor position: ", liftMotor1.getCurrentPosition());
    }
}