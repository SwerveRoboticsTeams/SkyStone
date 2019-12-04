package org.firstinspires.ftc.team8923_2019 ;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "CompetitionTeleOp")

public class CompetitionTeleOp extends MasterTeleOp
{
    @Override
    public void runOpMode()
    {

        initHardware();
        waitForStart();

        while (opModeIsActive())
        {

            driveMecanumTeleOp();

            sendTelemetry();
            toggleFoundationServos();
            runLift();
            runClaw();
            runCapstoneGrabber();
            runIntake();
            idle();

        }
    }

}
