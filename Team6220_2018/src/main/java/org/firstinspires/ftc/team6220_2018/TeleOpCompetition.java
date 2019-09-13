package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *  Uses driver input to operate robot during TeleOp period
 */

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeRobot();
        // Some motors are initialized here since they require RUN_TO_POSITION mode in autonomous,
        // but RUN_USING_ENCODER mode in TeleOp.
        motorHanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHanger.setPower(0);


        waitForStart();
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();


        // Main loop
        while (opModeIsActive())
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();


            // Drive methods
            driveMecanumWithJoysticks();
            driveHanger();
            driveArm();

            /*
             Updates that need to happen each loop
             Note:  eTime is not currently displayed (it interrupts other telemetry), but it may
             be useful later
            */
            //telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            //collectorEncoderState = collectorChannel.channelState;
            //telemetry.addData("Collector Channel: ", collectorEncoderState);
            telemetry.update();
            idle();
        }
    }
}
