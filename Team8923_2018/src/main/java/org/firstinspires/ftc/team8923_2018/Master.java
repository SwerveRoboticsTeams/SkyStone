package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

abstract class Master extends LinearOpMode
{
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorLift;
    DcMotor motorDankUnderglow;
    DcMotor motorFlip;
    CRServo motorSucc1;
    CRServo motorSucc2;
    //DcMotor motorFlip2;

    Servo servoJJ;
    Servo servoJJ2;

    BNO055IMU imu;

    double slowModeDivisor = 1.0;

    boolean reverseDrive = false;

    // Constants to be used in code. Measurements in millimeters
    private static final double GEAR_RATIO = 2.0/3.0; // Ratio of driven gear to driving gear
    private static final double TICKS_PER_MOTOR_REVOLUTION = 560.0;
    private static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION / GEAR_RATIO;
    private static final double WHEEL_DIAMETER = 4 * 25.4; // 4 inch diameter
    private static final double MM_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double MM_PER_TICK = MM_PER_REVOLUTION / TICKS_PER_WHEEL_REVOLUTION;
    static final double COUNTS_PER_MM = TICKS_PER_WHEEL_REVOLUTION / MM_PER_REVOLUTION;

    public void initHardware()
    {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        motorDankUnderglow = hardwareMap.get(DcMotor.class, "motorDankUnderglow");
        motorFlip = hardwareMap.get(DcMotor.class, "motorFlip");
        motorSucc1 = hardwareMap.get(CRServo.class, "motorSucc1");
        motorSucc2 = hardwareMap.get(CRServo.class, "motorSucc2");
        //motorFlip2 = hardwareMap.get(DcMotor.class, "motorFlip2");
        servoJJ = hardwareMap.get(Servo.class, "servoJJ");
        servoJJ2 = hardwareMap.get(Servo.class, "servoJJ2");

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorFlip2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorFlip2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFlip.setTargetPosition(motorFlip.getCurrentPosition());

        servoJJ2.setPosition(0.65);
    }



    void driveMecanum(double driveAngle, double drivePower, double turnPower)
    {
        // Calculate x and y components of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
        double x = drivePower * -Math.sin(Math.toRadians(driveAngle));
        double y = drivePower * Math.cos(Math.toRadians(driveAngle));

        /*
         * Below is an explanation of how the mecanum wheels are driven
         *
         * Each wheel has a roller on the bottom mounted at 45 degrees. Because of this, each wheel
         * can only exert a force diagonally in one dimension. Using the front left wheel as an
         * example, when it is given a positive power, it exerts a force forward and right, which
         * means positive y and x. When the front right wheel is given a positive power, it exerts
         * a force forward and left, which means positive y and negative x. This is reflected in
         * how the motor powers are set. Turning is like standard tank drive.
         */
        // Set motor powers
        if(reverseDrive)
            turnPower = -turnPower;

        double powerFL = y + x - turnPower;
        double powerFR = y - x + turnPower;
        double powerBL = y - x - turnPower;
        double powerBR = y + x + turnPower;

        // Motor powers might be set above 1, so this scales all of the motor powers to stay
        // proportional and within power range
        double scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        // Only apply scalar if greater than 1. Otherwise we could unintentionally increase power
        // This also prevents dividing by 0
        if(scalar < 1)
            scalar = 1;

        // Apply scalar
        powerFL /= (scalar * slowModeDivisor);
        powerFR /= (scalar * slowModeDivisor);
        powerBL /= (scalar * slowModeDivisor);
        powerBR /= (scalar * slowModeDivisor);

        if (!reverseDrive)
        {
            // Drive forwards
            motorFL.setPower(powerFL);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);
        }
        else
        {
            // Drive backwards
            motorFL.setPower(-powerFL);
            motorFR.setPower(-powerFR);
            motorBL.setPower(-powerBL);
            motorBR.setPower(-powerBR);
        }

    }

    boolean buttonsAreReleased(Gamepad pad)
    {
        return !(pad.a || pad.b || pad.x || pad.y || pad.left_bumper || pad.right_bumper
                || pad.dpad_up || pad.dpad_down || pad.dpad_left || pad.dpad_right
                || pad.left_stick_button || pad.right_stick_button
                || pad.start || pad.back || pad.guide || pad.left_trigger > 0.35
                || pad.right_trigger > 0.35);
    }

    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    // If you subtract 359 degrees from 0, you would get -359 instead of 1. This method handles
    // cases when one angle is multiple rotations away from the other
    double subtractAngles(double first, double second)
    {
        double delta = first - second;
        while(delta > 180)
            delta -= 360;
        while(delta <= -180)
            delta += 360;
        return delta;
    }
}
