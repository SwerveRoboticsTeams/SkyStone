package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;


/**
 * Program used to control Drive-A-Bots.
 * This can be a reference for drive controls.
 */
//@TeleOp(name="OmniDriveABot", group = "Swerve")
//@Disabled
public class OmniDriveABot extends LinearOpMode
{

    /*
        This omniwheel drivabot assumes the following "+" motor configuration:

             "front" of robot
                 motor1

        motor4            motor2

                 motor3
             "back" of robot



        Alternatively, you can configure your motors in an X pattern:

             "front" of robot

           motor1     motor2


           motor4     motor3

            "back" of robot



       The motor power calculations are different depending on whether you're using X or +.

       Here's an online video that explains how the programming works.
       https://www.youtube.com/watch?v=20M62Xil5s4

     */

    private enum OmniConfiguration
    {
        PLUS,
        X
    }

    //Change this line if your motors are in an X configuration
    private final OmniConfiguration myOmniBotConfiguration = OmniConfiguration.PLUS;


    //Omnidrive motors have a particular location on the robot.
    //The location is used to calculate the motor's power when driving.
    private class OmniMotor
    {
        DcMotor motor;
        int x, y, rotation;

        public OmniMotor(DcMotor aMotor, int aX, int aY, int aRotation)
        {
            motor = aMotor;
            x = aX;
            y = aY;
            rotation = aRotation;

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public double calculatePowerOmniPlusConfiguration(double requestedPower, double requestedRotation)
        {
            return requestedRotation + (Math.signum(x) * requestedPower);
        }

        public double calculatePowerOmniXConfiguration(double requestedX, double requestedY, double requestedRotation)
        {
            return  requestedRotation
                    + Math.signum(y) * requestedX
                    + Math.signum(x) * requestedY;
        }

        public void setPower(double power)
        {
            motor.setPower(power);
        }

        public double getPower()
        {
            return motor.getPower();
        }

    }


    OmniMotor motor1 = null;
    OmniMotor motor2 = null;
    OmniMotor motor3 = null;
    OmniMotor motor4 = null;


    @Override public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {

            driveOmniDrive( gamepad1.left_stick_x,    //local x motion power
                            gamepad1.left_stick_y,     //local y motion power
                            -gamepad1.right_stick_x / 2); //divide rotation in half so we don't spin too quickly

            telemetry.update();
            idle();
        }
    }


    public void driveOmniDrive(double x, double y, double rotation)
    {
        double power1=0, power2=0, power3=0, power4=0;

        if (myOmniBotConfiguration == OmniConfiguration.PLUS)
        {
            power1 = motor1.calculatePowerOmniPlusConfiguration(x, rotation);
            power2 = motor2.calculatePowerOmniPlusConfiguration(y, rotation);
            power3 = motor3.calculatePowerOmniPlusConfiguration(x, rotation);
            power4 = motor4.calculatePowerOmniPlusConfiguration(y, rotation);
        }
        else if (myOmniBotConfiguration == OmniConfiguration.X)
        {
            power1 = motor1.calculatePowerOmniXConfiguration(x, y, rotation);
            power2 = motor2.calculatePowerOmniXConfiguration(x, y, rotation);
            power3 = motor3.calculatePowerOmniXConfiguration(x, y, rotation);
            power4 = motor4.calculatePowerOmniXConfiguration(x, y, rotation);
        }

        //Find the maximum power applied to any motor
        double max = Math.max(Math.abs(power1), Math.abs(power2));
        max = Math.max(max, Math.abs(power3));
        max = Math.max(max, Math.abs(power4));

        //if any values were out of the motor power range of -1..1,
        // scale all of the values so they remain in proportion without overflowing
        if (max > 1.0)
        {
            power1 *= 1.0/max;
            power2  *= 1.0/max;
            power3  *= 1.0/max;
            power4   *= 1.0/max;
        }

        //set the powers
        motor1.setPower(power1);
        motor2.setPower(power2);
        motor3.setPower(power3);
        motor4.setPower(power4);

    }

    public void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        //configuration for "+"
        if (myOmniBotConfiguration == OmniConfiguration.PLUS)
        {
            motor1 = new OmniMotor(hardwareMap.dcMotor.get("motor1"), -1, 1, 0);
            motor2 = new OmniMotor(hardwareMap.dcMotor.get("motor2"), -1, -1, 90);
            motor3 = new OmniMotor(hardwareMap.dcMotor.get("motor3"), 1, -1, 180);
            motor4 = new OmniMotor(hardwareMap.dcMotor.get("motor4"), 1, 1, 270);
        }
        else if (myOmniBotConfiguration == OmniConfiguration.X)
        {
            //configuration for "X"
            //adjust these 1's and -1's if your robot doesn't move in the direction you like
            motor1 = new OmniMotor(hardwareMap.dcMotor.get("motor1"),  1,   1, 315);
            motor2 = new OmniMotor(hardwareMap.dcMotor.get("motor2"),  1,  -1,  45);
            motor3 = new OmniMotor(hardwareMap.dcMotor.get("motor3"), -1,  -1, 135);
            motor4 = new OmniMotor(hardwareMap.dcMotor.get("motor4"), -1,   1, 225);
        }
        // Set up telemetry data
        configureDashboard();
    }

    public void configureDashboard()
    {
        telemetry.addLine()
                .addData("Power | 1: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor1.getPower());
                    }
                })
                .addData("2: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor2.getPower());
                    }
                })
                .addData("3: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor3.getPower());
                    }
                })
                .addData("4: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor4.getPower());
                    }
                });
    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
