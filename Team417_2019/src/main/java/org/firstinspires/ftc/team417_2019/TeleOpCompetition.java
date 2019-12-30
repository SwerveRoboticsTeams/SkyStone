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
            mecanumDrive();
            linearSlides();
            collector();
            foundationPullers();
            updateTelemetry();

            if (gamepad1.a) {
                telemetry.addData("core2", core2.getCurrentPosition());
                telemetry.addData("Main Wrist", mainWristServo.getPosition());
                telemetry.addData("Arm 1", arm1.getCurrentPosition());
                updateTelemetry();

            }
        }
    }
}
