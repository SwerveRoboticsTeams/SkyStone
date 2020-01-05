package org.firstinspires.ftc.team6220_2019.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_2019.ResourceClasses.Button;

@TeleOp(name = "Lift Motor Test")
public class LiftMotorTest extends LiftMotorTestMode
{
    double speedUp = 0.5;
    double speedDown = 0.2;
    boolean toggleUp = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        waitForStart();
        double lTime = timer.seconds();

        while (opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            telemetry.addData("Speed up: ", speedUp);
            telemetry.addData("Speed down: ", speedDown);
            telemetry.addData(toggleUp ? "Currently updating up" : "Currently updating down", "");
            telemetry.update();
            if (driver.isButtonPressed(Button.DPAD_UP))
            {
                liftMotor0.setPower(-speedUp);
                liftMotor1.setPower(speedUp);
            } else if (driver.isButtonPressed(Button.DPAD_DOWN))
            {
                liftMotor0.setPower(speedDown);
                liftMotor1.setPower(-speedDown);
            } else
            {
                liftMotor0.setPower(0);
                liftMotor1.setPower(0);
            }
            if (driver.isButtonJustPressed(Button.A))
            {
                toggleUp = !toggleUp;
            }
            if (driver.isButtonJustPressed(Button.RIGHT_BUMPER))
            {
                if (toggleUp && speedUp < 1)
                {
                    speedUp += 0.1;
                } else if (!toggleUp && speedDown < 1)
                {
                    speedDown += 0.1;
                }
            }
            if (driver.isButtonJustPressed(Button.LEFT_BUMPER))
            {
                if (toggleUp && speedUp > 0)
                {
                    speedUp -= 0.1;
                } else if (!toggleUp && speedDown > 0)
                {
                    speedDown -= 0.1;
                }
            }

            updateCallback(eTime);
            idle();
        }
    }
}
