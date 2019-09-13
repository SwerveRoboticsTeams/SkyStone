package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
    Contains methods for accepting and interpreting pilot and co-pilot input.
    Note:  extends MasterAutonomous in order to have access to navigation / turning functionality.
*/
abstract public class MasterTeleOp extends MasterAutonomous
{
    // Start robot in slow mode (per Slater)
    boolean slowMode = true;
    // Allows us to switch front of robot.
    boolean driveReversed = true;
    // Allows us to switch front of robot.
    boolean hangServoDeployed = false;
    // Determines whether arm is in RUN_TO_POSITION (=false) or RUN_USING_ENCODER (=true).
    boolean armRunModeUsingEncoder = false;

    boolean collectorEncoderState = false;

    double time = getRuntime();

    // Stores position of arm
    double armPos = 0;

    /*
    // Collector operation booleans
    boolean collectorSlowMode = false;
    boolean isCollectorStopping = false;
    boolean isCollecting = false;
    boolean isCollectingIn = false;
    // Allows us to break out of collector encoder loop if necessary.
    ElapsedTime collectorLoopTimer = new ElapsedTime();

    double collectorPowerIn = Constants.MOTOR_COLLECTOR_IN;
    double collectorPowerOut = Constants.MOTOR_COLLECTOR_OUT;
    */

    // Factor that adjusts magnitudes of vertical and horizontal movement.
    double tFactor = Constants.SLOW_MODE_T_FACTOR;
    // Factor that adjusts magnitude of rotational movement.
    double rFactor = Constants.SLOW_MODE_R_FACTOR;

    // Takes driver 1 input to run hanger system.
    void driveHanger()
    {
        if (driver1.getRightTriggerValue() >= Constants.MINIMUM_TRIGGER_VALUE)
            motorHanger.setPower(stickCurve.getOuput(-driver1.getRightTriggerValue()));
        else if (driver1.getLeftTriggerValue() >= Constants.MINIMUM_TRIGGER_VALUE)
            motorHanger.setPower(stickCurve.getOuput(driver1.getLeftTriggerValue()));
        else if (motorHanger.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            motorHanger.setPower(0.0);

        // Toggle hanger servo
        if (driver1.isButtonJustPressed(Button.B) && !hangServoDeployed)
        {
            servoHanger.setPosition(Constants.SERVO_HANG_DEPLOYED);
            hangServoDeployed = true;
        }
        else if (driver1.isButtonJustPressed(Button.B) && hangServoDeployed)
        {
            servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
            hangServoDeployed = false;
        }

        // Run hanger to automatic height of driver 1 presses a.
        if (driver1.isButtonJustPressed(Button.A))
        {
            motorHanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorHanger.setPower(1.0);
            motorHanger.setTargetPosition(Constants.HANG_TELEOP_HEIGHT);
        }
        // Exit if hanger is at position.
        else if ((motorHanger.getMode() == DcMotor.RunMode.RUN_TO_POSITION) && (motorHanger.getCurrentPosition() - Constants.HANG_TELEOP_HEIGHT < 50))
        {
            motorHanger.setPower(0);
            motorHanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // Also exit if driver 1 presses y.
        else if (driver1.isButtonJustPressed(Button.Y))
        {
            motorHanger.setPower(0);
            motorHanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //telemetry.addData("Trigger val Right: ", driver1.getRightTriggerValue());
        //telemetry.addData("Trigger val Left: ", driver1.getLeftTriggerValue());
        //telemetry.addData("Hanger Enc: ", motorHanger.getCurrentPosition());
        //telemetry.update();
    }

    // Uses driver 2 input to drive arm and collector motors.
    void driveArm()
    {
        // Run arm to stick power.
        if (Math.abs(driver2.getRightStickY()) >= Constants.MINIMUM_JOYSTICK_POWER)
        {
            armRunModeUsingEncoder = true;
            motorArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armPos = (motorArmLeft.getCurrentPosition() - motorArmRight.getCurrentPosition()) / 2.0;

            /*
            Use a normal distribution to speed up arm motion in middle of range and slow it down
            near edges of range.  Also set arm to oonstant power if it gets outside this range
            so motion is not unnecessarily slow.
            */
            if (armPos >= Constants.ARM_GROUND - 100 && armPos <=Constants.ARM_TOP_BLOCKS + 100)
            {
                powerArm(-Constants.HIGH_ARM_POWER * Math.pow(1.5, -Math.pow((armPos - 600) / 400, 2)) * stickCurve.getOuput(driver2.getRightStickY()));
            }
            else
            {
                powerArm(-Constants.LOW_ARM_POWER * stickCurve.getOuput(driver2.getRightStickY()));
            }
        }
        // Change to RUN_TO_POSITION when stick is not pressed.
        else if (Math.abs(driver2.getRightStickY()) < Constants.MINIMUM_JOYSTICK_POWER && armRunModeUsingEncoder)
        {
            armRunModeUsingEncoder = false;
            powerArm(0);
            motorArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // Use RUN_TO_POSITION.
        else if (Math.abs(driver2.getRightStickY()) < Constants.MINIMUM_JOYSTICK_POWER && !armRunModeUsingEncoder)
        {
            if (driver2.isButtonPressed(Button.Y))
            {
                motorArmLeft.setTargetPosition(Constants.ARM_TOP_BLOCKS);
                motorArmRight.setTargetPosition(-Constants.ARM_TOP_BLOCKS);
                armPos = motorArmLeft.getCurrentPosition();

                if (armPos <= Constants.ARM_HIGH_SWITCH_HEIGHT)
                {
                    powerArm(Constants.HIGH_ARM_POWER);
                }
                else
                {
                    powerArm(Constants.LOW_ARM_POWER);
                }
            }

            if(driver2.isButtonPressed(Button.X))
            {
                motorArmLeft.setTargetPosition(Constants.ARM_TOP_BALLS);
                motorArmRight.setTargetPosition(-Constants.ARM_TOP_BALLS);
                if (motorArmLeft.getCurrentPosition() <= Constants.ARM_HIGH_SWITCH_HEIGHT)
                {
                    powerArm(Constants.HIGH_ARM_POWER);
                }
                else
                {
                    powerArm(Constants.LOW_ARM_POWER);
                }
            }
        }


        telemetry.addData("Arm Position Left: ", motorArmLeft.getCurrentPosition());
        telemetry.addData("Arm Position Right: ", motorArmRight.getCurrentPosition());
        //telemetry.addData("Arm Run Mode Using Encoder: ", armRunModeUsingEncoder);
        //telemetry.addData("Collector Slow Mode: ", collectorSlowMode);
        telemetry.update();
    }

    // Takes driver 1 stick input and uses it to give power and direction inputs to the drive
    void driveMecanumWithJoysticks()
    {
        // Note: factors are different for translation and rotation
        // Slow mode functionality.  1st driver presses right bumper to toggle slow mode
        if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER) && !slowMode)
        {
            tFactor = Constants.SLOW_MODE_T_FACTOR;
            rFactor = Constants.SLOW_MODE_R_FACTOR;
            slowMode = true;
        }
        else if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER) && slowMode)
        {
            tFactor = Constants.T_FACTOR;
            rFactor = Constants.R_FACTOR;
            slowMode = false;
        }


        // Stick inputs must be changed from x and y to angle, drive power, and rotation power---
        double angle = Math.toDegrees(driver1.getRightStickAngle());

        double drivePower = tFactor * stickCurve.getOuput(driver1.getRightStickMagnitude());

        double rotationPower = -rFactor * stickCurve.getOuput(gamepad1.left_stick_x);
        //----------------------------------------------------------------------------------------


        /*
        // Change drive direction based on driver input
        if (driver1.isButtonJustPressed(Button.LEFT_BUMPER) && !driveReversed)
            driveReversed = true;
        else if (driver1.isButtonJustPressed(Button.LEFT_BUMPER) && driveReversed)
            driveReversed = false;
        */

        // Drive in direction based on whether driveDirectionShift is true
        if (!driveReversed)
            driveMecanum(angle, drivePower, rotationPower);
        else
            driveMecanum(angle + 180, drivePower, rotationPower);
    }
}
