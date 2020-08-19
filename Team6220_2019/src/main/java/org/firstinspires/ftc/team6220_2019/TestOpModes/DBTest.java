package org.firstinspires.ftc.team6220_2019.TestOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "DBTest", group = "TeleOp")
public class DBTest extends LinearOpMode
{

    //Motors
    DcMotor motorLeft;
    DcMotor motorRight;

    //Other Devices
    BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException
    {

        //Initialize
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Init", "Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            if (Math.abs(gamepad1.left_stick_y) > 0.02 || Math.abs(gamepad1.right_stick_y) > 0.02)
            {
                motorLeft.setPower(-((4/5.0) * (0.5 * Math.pow(gamepad1.left_stick_y,  3)) + (0.5 * gamepad1.left_stick_y)));
                motorRight.setPower(-((4/5.0) * (0.5 * Math.pow(gamepad1.right_stick_y, 3)) + (0.5 * gamepad1.right_stick_y)));
            }
            /*else if(gamepad1.a)
            {
                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);
                motorLeft.setTargetPosition(1120 * 1);
                motorRight.setTargetPosition(1120 * 1);
                motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }*/
            else
            {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }
            //Telemetry
            telemetry.addData("MotorLeftPower: ", -motorLeft.getPower());
            telemetry.addData("MotorRightPower: ", -motorRight.getPower());
            telemetry.addData("EncoderLeft: ", motorLeft.getCurrentPosition());
            telemetry.addData("EncoderRight: ", motorRight.getCurrentPosition());

            telemetry.addData("IMU Angle: ", imu.getAngularOrientation().firstAngle);
            telemetry.addData("IMU x Acceleration: ", imu.getLinearAcceleration().xAccel);
            telemetry.addData("IMU y Acceleration: ", imu.getLinearAcceleration().yAccel);
            telemetry.update();

            idle();
        }
    }
}
