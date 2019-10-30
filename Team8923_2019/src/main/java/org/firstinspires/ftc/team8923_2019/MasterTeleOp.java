package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;



abstract class MasterTeleOp extends Master
{

    public void driveMecanumTeleOp()
    {
        // Reverse drive if desired
        if(gamepad1.right_stick_button)
            reverseDrive = false;
        if(gamepad1.left_stick_button)
            reverseDrive = true;

        if(gamepad1.dpad_down)
            slowModeDivisor = 3.0;
        else if(gamepad1.dpad_up)
            slowModeDivisor = 1.0;

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = 0.5 * gamepad1.left_stick_x;
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation
        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveMecanum(angle, power, turnPower);
    }

    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    public void runIntake()
    {
        if (gamepad1.dpad_up)
        {
            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);

        }
        else if (gamepad1.dpad_down)
        {
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(-1.0);
        }
        else
        {
            intakeLeft.setPower(0.0);
            intakeRight.setPower(0.0);
        }
//        telemetry.addData("intake", gamepad1.dpad_up);
//        telemetry.update();

    }




    public void runClaw()
    {
        double leftStickY = gamepad2.left_stick_y;

        if(leftStickY > Constants.MINIMUM_JOYSTICK_PWR)
        {
            // Move arm forward
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setPower(leftStickY * Constants.ARM_PWR_FACTOR);
        }
        else if(leftStickY < -Constants.MINIMUM_JOYSTICK_PWR)
        {
            // Move arm back
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setPower(leftStickY * Constants.ARM_PWR_FACTOR);
        }
        else
        {
            // Stops arm
            motorArm.setTargetPosition(motorArm.getCurrentPosition());
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Fix arm dropping

        }


    }

    public void adjustClawPosition()
    {
        // Make claw level to ground
        if(Variables.ARM_MOTOR_TICKS < Constants.MAX_ENCODERCOUNT_PARALLEL_POINT)
        {
            double adjustedServoPosition = map(Variables.ARM_MOTOR_TICKS, Constants.MIN_ENCODERCOUNT_PARALLEL_POINT,Constants.MAX_ENCODERCOUNT_PARALLEL_POINT, Constants.MIN_SERVOJOINT_PWR, Constants.MAX_SERVOJOINT_PWR);
            servoJoint.setPosition(adjustedServoPosition);
        }
    }

    public void toggleGrabber()
    {
        double rightTrigger = gamepad2.right_trigger;
        if(rightTrigger > Constants.MINIMUM_TRIGGER_VALUE){
            servoGrabber.setPosition(rightTrigger*0.9);
        }else{
            servoGrabber.setPosition(0.5);
        }

    }

    private double map(double value, double minInput, double maxInput, double minMappedOutput, double maxMappedOutput)
    {
        double valueDifference = maxInput - minInput;
        double percentValueDifference = (value - minInput) / valueDifference;
        double mappedDifference = maxMappedOutput - minMappedOutput;

        return percentValueDifference * mappedDifference + minMappedOutput;
    }




    void sendTelemetry()
    {

        telemetry.addData("left stick x:", gamepad1.left_stick_x);
        telemetry.addData("left stick y:", gamepad1.left_stick_y);
        telemetry.addData("right stick x:", gamepad1.right_stick_x);
        telemetry.addData("right stick y:", gamepad1.right_stick_y);
        telemetry.addData("right trigger:", gamepad2.right_trigger);
        telemetry.addData("left trigger:", gamepad2.left_trigger);
        telemetry.addData("armStartingTicks", Constants.ARM_STARTING_TICKS);
        telemetry.addData("armTicks", Variables.ARM_MOTOR_TICKS);
        telemetry.update();
    }
}
