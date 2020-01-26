package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

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
            slowModeDivisor = 1.0;

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
            servoCapstone.setPosition(.39);
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
            //servoMoveTo(servoJoint,1,95);
            servoJoint.setPosition(1);
            clawIsOut = false;
        }else if(gamepad2.a){
            //servoMoveTo(servoJoint,0.03,95);
            servoJoint.setPosition(0);
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

        telemetry.addData("lift ticks", motorLift2.getCurrentPosition());
        //telemetry.addData("first", imu.getAngularOrientation().firstAngle);
        //telemetry.addData("second", imu.getAngularOrientation().secondAngle);
        //telemetry.addData("Third", imu.getAngularOrientation().thirdAngle);
//
//        telemetry.addData("left stick x:", gamepad1.left_stick_x);
//        telemetry.addData("left stick y:", gamepad1.left_stick_y);
//        telemetry.addData("right stick x:", gamepad1.right_stick_x);
//        telemetry.addData("right stick y:", gamepad1.right_stick_y);
//        telemetry.addData("right trigger:", gamepad2.right_trigger);
//        telemetry.addData("left trigger:", gamepad2.left_trigger);


        telemetry.update();
    }

    // Helper Functions
    // todo Ask how to extend the servo class so we don't have to pass in servo
    private void servoMoveTo(Servo servo, double targetPosition, double percentSpeed)
    {
        try {
            // Get current servo position
            double servoCurrentPosition = servo.getPosition();

            // Map servo position to degrees
            int servoPositionInDegrees = (int)map(servoCurrentPosition,0,1,0,180);

            // Map target position to degrees
            double targetPositionInDegrees = map(targetPosition,0,1,0,180);

            // Calculate delay from percent speed
            // -increase minMappedOutput for slower speeds
            double delay = map(percentSpeed, 0, 100,5,0);

            // Calculate increment for loop (target position could be higher or lower than current position)
            int increment;
            if(servoPositionInDegrees < targetPositionInDegrees){
                increment = 1;
            }else{
                increment = -1;
            }

            // Move to each degree with calculated delay
            for(int i = servoPositionInDegrees; i != targetPositionInDegrees; i += increment){
                double servoPositionZeroToOne = map(i,0,180,0,1);
                servo.setPosition(servoPositionZeroToOne);
                sleep((int)delay);
            }

        } catch (Exception e) {
            System.out.printf("no servo");
        }
    }

    private double map(double value, double minInput, double maxInput, double minMappedOutput, double maxMappedOutput)
    {
        double valueDifference = maxInput - minInput;
        double percentValueDifference = (value - minInput) / valueDifference;
        double mappedDifference = maxMappedOutput - minMappedOutput;

        return percentValueDifference * mappedDifference + minMappedOutput;
    }
}
