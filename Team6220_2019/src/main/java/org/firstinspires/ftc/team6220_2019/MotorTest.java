package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor Test")
public class MotorTest extends MasterOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Add gamepad to callback---------------
        driver1 = new DriverInput(gamepad1);
        callback.add(driver1);

        int newTargetPos = 5000;


        DcMotor motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Have to set this BEFORE RUN_TO_POSITION now
        motor.setTargetPosition(newTargetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();

        motor.setPower(0.6);

        while(/*motor.isBusy()*/ opModeIsActive())
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();


            if (driver1.isButtonJustPressed(Button.DPAD_UP))
            {
                newTargetPos += 1000;
                motor.setTargetPosition(newTargetPos);
            }
            else if (driver1.isButtonJustPressed(Button.DPAD_DOWN))
            {
                newTargetPos -= 1000;
                motor.setTargetPosition(newTargetPos);
            }


            telemetry.addData("Motor Power: ", motor.getPower());
            telemetry.addData("Motor Position: ", motor.getCurrentPosition());
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}