package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        initializeHardware();

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Init:", "Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            driveRobot();
            linearSlides();
            collector();
            foundationPullers();
            updateTelemetry();
            if(gamepad2.x){
                lower();
            }

                telemetry.addData("core2", core2.getCurrentPosition());
                telemetry.addData("Main Wrist", mainWristServo.getPosition());
                telemetry.addData("Arm 1", arm1.getCurrentPosition());
                telemetry.addData("Gamepad 2", gamepad2.right_stick_y);
                updateTelemetry();
        }
    }
}
