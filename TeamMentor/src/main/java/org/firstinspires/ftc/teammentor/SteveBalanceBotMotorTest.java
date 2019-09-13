package org.firstinspires.ftc.teammentor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * Program used to control a self-balancing robot on two wheels.
 */
@TeleOp(name="BalanceBot Test", group = "Steve")
@Disabled
public class SteveBalanceBotMotorTest extends SteveBalancebotMaster
{


    @Override public void runOpMode() throws InterruptedException
    {
        double error = 0;
        double motorSpeed = 0.0;

        // Initialize hardware and other important things
        initializeRobot();

        //Configure telemetry, if we want it on
        //configureDashboard();

        // Wait until start button has been pressed
        waitForStart();

        //wait for IMU to start running
        while (imu.getSystemStatus() != BNO055IMU.SystemStatus.RUNNING_FUSION)
        {
            telemetry.addData("init", "starting imu...");
            telemetry.addData("state", imu.getSystemStatus());
            idle();
        }

        // Main loop
        while(opModeIsActive())
        {

            motorLeft.setPower(gamepad1.left_stick_y);
            motorRight.setPower(gamepad1.right_stick_y);

            telemetry.addData("left encoder", motorLeft.getCurrentPosition());
            telemetry.addData("right encoder", motorRight.getCurrentPosition());

            telemetry.update();
            idle();
        }
    }


    public void configureDashboard()
    {

        telemetry.addLine()
                .addData("Left", new Func<String>() {
                    @Override public String value() {
                return formatNumber(motorLeft.getCurrentPosition());
                }
                })
                .addData("Right", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorRight.getCurrentPosition());
                    }
                });
    }

    public String formatNumber(double d)
    {
        return String.format("%.4f", d);
    }
    public String formatNumberEightDigits(double d)
    {
        return String.format("%.8f", d);
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
}
