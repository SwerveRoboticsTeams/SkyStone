package org.firstinspires.ftc.teammentor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * Program used to control a self-balancing robot on two wheels.
 */
@TeleOp(name="BalanceBot", group = "Steve")
// @Disabled
public class SteveBalanceBot extends SteveBalancebotMaster
{

    @Override public void runOpMode() throws InterruptedException
    {
        double error = 0;
        double motorSpeed = 0.0;
        boolean emergencyStop = false;

        ElapsedTime time = new ElapsedTime();

        FileWriter fileWriter = new FileWriter("balancebot.csv");

        // Initialize hardware and other important things
        initializeRobot();

        //Configure telemetry, if we want it on
        //configureDashboard();

        //wait for IMU to start running
        while (imu.getSystemStatus() != BNO055IMU.SystemStatus.RUNNING_FUSION)
        {
            telemetry.addData("init", "starting imu...");
            telemetry.addData("state", imu.getSystemStatus());
            idle();
        }

        // Wait until start button has been pressed
        waitForStart();

        time.reset();


        // Main loop
        while(opModeIsActive() && !emergencyStop)
        {

            //some gamepad controls that may help me tune the PID constants
            if (gamepad1.a)
            {
                while (gamepad1.a) {}
                P_CONSTANT += 0.002;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.b)
            {
                while (gamepad1.b) {}
                P_CONSTANT -= 0.002;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.x)
            {
                while (gamepad1.x) {}
                P_CONSTANT += 0.01;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.y)
            {
                while (gamepad1.y) {}
                P_CONSTANT -= 0.01;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.dpad_up)
            {
                while (gamepad1.dpad_up) {}
                D_CONSTANT += 0.001;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.dpad_down)
            {
                while (gamepad1.dpad_down) {}
                D_CONSTANT -= 0.001;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.left_bumper)
            {
                telemetry.addData("KP", P_CONSTANT);
                telemetry.addData("KI", I_CONSTANT);
                telemetry.addData("KD", D_CONSTANT);
            }



            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //currentRoll = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));
            currentRoll = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle);
            //telemetry.addData("roll", currentRoll);

            if (Math.abs(currentRoll) > MAX_SAFE_ROLL)
            {
                emergencyStop = true;
                motorLeft.setPower(0);
                motorRight.setPower(0);
                telemetry.addData("estop", "estop");
                fileWriter.closeFile();

                //opmode will end now due to emergencyStop being true
                while (true)
                {
                    telemetry.addData("PID",
                            formatPID(filterPID.getP()) + "," +
                                    formatPID(filterPID.getI()) + "," +
                                    formatPID(filterPID.getD()));
                    telemetry.update();
                    idle();
                }

            }
            else {
                error = targetRoll - currentRoll;

                motorSpeed = filterPID.getFilteredValue(error);
                //telemetry.addData("speed", motorSpeed);

                String p = formatPID(filterPID.getP()) + ",";
                String i = formatPID(filterPID.getI()) + ",";
                String d = formatPID(filterPID.getD());

                fileWriter.println(time.milliseconds() + "," +
                        error + "," +
                        formatNumberEightDigits(motorSpeed) +
                        p + i + d);

                telemetry.addData("error", error);
                telemetry.addData("PID",
                        formatPID(filterPID.getP()) + ",  " +
                                formatPID(filterPID.getI()) + ",  " +
                                formatPID(filterPID.getD()));

                motorLeft.setPower(motorSpeed);
                motorRight.setPower(motorSpeed);
            }

            telemetry.update();
            idle();
        }
    }


    public void configureDashboard()
    {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        //this is no longer needed since I get this as part of my main loop
        /*
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });
        */

        /*
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
         */

        telemetry.addData("currRoll", new Func<String>() {
            @Override public String value() {
                return formatDegrees(currentRoll);
            }
        });

        telemetry.addLine()
                .addData("P", new Func<String>() {
                    @Override public String value() {
                return formatNumber(P_CONSTANT);
                }
                })
                .addData("I", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(I_CONSTANT);
                    }
                })
                .addData("D", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(D_CONSTANT);
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

    String formatPID(double factor){
        return String.format("%.8f", factor);
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
}
