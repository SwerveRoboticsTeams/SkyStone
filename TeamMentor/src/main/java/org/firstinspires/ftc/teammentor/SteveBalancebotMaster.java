package org.firstinspires.ftc.teammentor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * Program used to control a self-balancing robot on two wheels.
 */
//@TeleOp(name="BalanceBot", group = "Steve")
// @Disabled
abstract public class SteveBalancebotMaster extends LinearOpMode
{
    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    //imu sensor object
    BNO055IMU imu = null;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    //The IMU emits heading ("first angle"), roll ("second angle"), and pitch ("third angle").
    //On my new robot, rooll determines whether the robot is balanced.
    double targetRoll = 0.0;

    double currentRoll = 0.0;

    FilterPID filterPID = null;


    //notes about P constant
    /*
     *  NEVEREST 20's
     *  (new design, no data)
     *
    */

    //These PID constants assume Neverest 20's and my second iteration of the robot design.
    public double P_CONSTANT = 0.016;
    public double I_CONSTANT = 0.0000;
    public double D_CONSTANT = 0.006;


    /* If the robot tips over too far, it's not really recoverable.
     * Let's define a constant that is a fail-safe to stop the motors if the robot has tipped too far.
     * My manual tipping of the robot suggests that up to 10 may be fine, and 20 is certainly "too far".
    */
    double MAX_SAFE_ROLL = 25.0;


    public void initializeRobot()
    {
        //Connect our motor member variables to the hardware motors
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        filterPID = new FilterPID(this, P_CONSTANT, I_CONSTANT, D_CONSTANT);
    }

}
