package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        waitForStart();
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();

        while(opModeIsActive())
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();


            // Operate robot mechanisms using methods from MasterTeleOp
             // Driver1 methods
            driveMecanumWithJoysticks();
             // Driver2 methods
            driveCollector();
            driveScoringSystem();
            // Only toggle grabber if Button.A is just pressed.
            if(driver2.isButtonJustPressed(Button.RIGHT_BUMPER))
                toggleGrabber();


            /*
             Updates that need to happen each loop
             Note:  eTime is not currently displayed (it interrupts other telemetry), but it may
             be useful later
            */
            //telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            //collectorEncoderState = collectorChannel.channelState;
            //telemetry.addData("Collector Channel: ", collectorEncoderState);
            telemetry.addData("Current Tower Height: ", towerHeight);
            telemetry.update();
            idle();
        }
    }
}