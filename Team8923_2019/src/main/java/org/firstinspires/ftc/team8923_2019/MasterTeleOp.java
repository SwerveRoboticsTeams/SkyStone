package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.VariableSizeInsn;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;



abstract class MasterTeleOp extends Master

{
    boolean clawIsOut = false;
    boolean capstoneIsUp = true;

    /*
        gamepad 1 controls
        a = reverse drive
        b = forward drive
        right trigger toggle foundation pullers

        gamepad 2 controls
        y to capstone
        d pad up and down to intake
        left stick up and down for lift
        a for claw out
        b for claw in
    */


    public void driveMecanumTeleOp()
    {
        // Reverse drive if desired
        if(gamepad1.b)
            reverseDrive = true;
        if(gamepad1.a)
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

    public void toggleFoundationServos()
    {
        if(gamepad1.left_trigger > Constants.MINIMUM_TRIGGER_VALUE){
            //down
            servoFoundationLeft.setPosition(Constants.LEFT_FOUNDATION_SERVO_POSITION_DOWN);
            servoFoundationRight.setPosition(Constants.RIGHT_FOUNDATION_SERVO_POSITION_DOWN);
        }else{
            //up
            servoFoundationLeft.setPosition(Constants.LEFT_FOUNDATION_SERVO_POSITION_UP);
            servoFoundationRight.setPosition(Constants.RIGHT_FOUNDATION_SERVO_POSITION_UP);
        }
    }

    public void runCapstonePlacer()
    {
        if (gamepad2.y) {
            servoCapstone.setPosition(0.0);

        }else if (gamepad2.x){
            servoCapstone.setPosition(.44);
        }

    }

    public void runIntake()
    {
        if(gamepad1.right_bumper){
            intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeLeft.setPower(Variables.INTAKE_PWR);
            intakeRight.setPower(Variables.INTAKE_PWR);
        }else if(gamepad1.right_trigger > Constants.MINIMUM_TRIGGER_VALUE){
            intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeLeft.setPower(Variables.INTAKE_PWR * gamepad1.right_trigger);
            intakeRight.setPower(Variables.INTAKE_PWR * gamepad1.right_trigger);
        }else{
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }

    }

    public void runLift()
    {
        if(gamepad2.left_stick_y < -Constants.MINIMUM_JOYSTICK_PWR){
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLift.setPower(gamepad2.left_stick_y * Constants.LIFT_PWR);
            motorLift2.setPower(gamepad2.left_stick_y * Constants.LIFT_PWR);

            //motorLift.setTargetPosition(motorLift.getCurrentPosition());

        }else if(gamepad2.left_stick_y > Constants.MINIMUM_JOYSTICK_PWR){
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLift.setPower(gamepad2.left_stick_y * Constants.LIFT_PWR);
            motorLift2.setPower(gamepad2.left_stick_y * Constants.LIFT_PWR);

            //motorLift.setTargetPosition(motorLift.getCurrentPosition());

        }else{
            motorLift2.setPower(0);
            //motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //double pwr = motorLift.getPower();
            motorLift.setPower(0);

        }
    }

    public void runJoint(){
        if(gamepad2.b){
            servoJoint.setPosition(1);
            clawIsOut = false;
        }else if(gamepad2.a){
            servoJoint.setPosition(0.03);
            clawIsOut = true;

        }
    }

    public void runClaw(){
        if(gamepad2.right_trigger > Constants.MINIMUM_TRIGGER_VALUE){
            servoClaw.setPosition(0);
        }else{
            servoClaw.setPosition(0.35);
        }
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

    // Helper Functions
    private double map(double value, double minInput, double maxInput, double minMappedOutput, double maxMappedOutput)
    {
        double valueDifference = maxInput - minInput;
        double percentValueDifference = (value - minInput) / valueDifference;
        double mappedDifference = maxMappedOutput - minMappedOutput;

        return percentValueDifference * mappedDifference + minMappedOutput;
    }
}
