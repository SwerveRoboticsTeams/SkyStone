package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp Competition")

public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode()
    {
        initHardware();
        servoJJ.setPosition(0.0);
        motorFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive())
        {
            //dankUnderglow(-1.0);
            driveMecanumTeleOp();
            runLift();
            fastFlex();
            runFlipNSuccNPush();
            sendTelemetry();
            idle();
        }
    }
}
