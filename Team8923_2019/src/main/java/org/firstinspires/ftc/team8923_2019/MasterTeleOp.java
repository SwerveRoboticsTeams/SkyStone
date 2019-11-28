package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;



abstract class MasterTeleOp extends Master
{

    public void driveMecanumTeleOp()
    {
        // Reverse drive if desired
        if(gamepad1.a)
            reverseDrive = true;
        if(gamepad1.b)
            reverseDrive = false;

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

    public void toggleFoundationServos(){
        if(gamepad1.right_trigger > Constants.MINIMUM_TRIGGER_VALUE){
            servoFoundationLeft.setPosition(0.0);
            servoFoundationRight.setPosition(1.0);
        }else{
            servoFoundationLeft.setPosition(0.7);
            servoFoundationRight.setPosition(0);
        }
    }

    public void runCapstoneGrabber(){
        if (gamepad2.y) {
            servoCapstone.setPosition(0.0);
        }else {
            servoCapstone.setPosition(0.55);
        }


    }

    public void runIntake(){
        if(gamepad2.dpad_down){
            intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeLeft.setPower(Constants.INTAKE_PWR);
            intakeRight.setPower(Constants.INTAKE_PWR);
        }else if(gamepad2.dpad_up){
            intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeLeft.setPower(Constants.INTAKE_PWR);
            intakeRight.setPower(Constants.INTAKE_PWR);
        }else{
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
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


        telemetry.update();
    }
}
