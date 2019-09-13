package org.firstinspires.ftc.teammentor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;


/**
 * Program used to control a self-balancing robot on two wheels.
 */
@TeleOp(name="BalanceBot Hardware Test", group = "Steve")
// @Disabled
public class SteveBalanceBotHardwareTest extends SteveBalancebotMaster
{


    @Override public void runOpMode() throws InterruptedException
    {
        double error = 0;
        double motorMove = 0.5;
        double motorStop = 0.0;

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

            //motorLeft.setPower(motorMove);
            //motorRight.setPower(motorMove);

            if (gamepad1.a) {
                motorLeft.setPower(motorMove);
                motorRight.setPower(motorMove);
            }

            if (gamepad1.b) {
                motorLeft.setPower(motorStop);
                motorRight.setPower(motorStop);
            }

           // telemetry.addData("left encoder", motorLeft.getCurrentPosition());
           // telemetry.addData("right encoder", motorRight.getCurrentPosition());



            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //gravity  = imu.getGravity();
            //telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
            //telemetry.addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
            /*
            telemetry.addData("grvty", gravity.toString());
            telemetry.addData("mag", String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel*gravity.xAccel
                                            + gravity.yAccel*gravity.yAccel
                                            + gravity.zAccel*gravity.zAccel)));
            */

            telemetry.update();
            idle();
        }
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
