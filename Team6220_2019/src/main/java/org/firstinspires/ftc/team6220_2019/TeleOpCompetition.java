package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_2019.ResourceClasses.Button;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // todo Use Vuforia for semiautonomous functionality in future
        initialize(false);

        waitForStart();
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();

        // Grabber arn must initialize after match starts in order to satisfy 18" size constraint, after which it flips out collector.
        grabberArmLeft.setPosition(Constants.GRABBER_ARM_SERVO_LEFT_RETRACT);
        grabberArmRight.setPosition(Constants.GRABBER_ARM_SERVO_RIGHT_RETRACT);


        while(opModeIsActive())
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();
            
            // Operate robot mechanisms using methods from MasterTeleOp-----------------------------
             // Driver 1 controls
            driveMecanumWithJoysticks();
            driveCollector();

            // Driver 2 controls
            driveLift();
             // Only toggle foundationServos if Button.A is just pressed.
            if(driver2.isButtonJustPressed(Button.A))
                toggleFoundationServos();
            //--------------------------------------------------------------------------------------

            /*
             Updates that need to happen each loop
             Note:  eTime is not currently displayed (it interrupts other telemetry), but it may
             be useful later
            */
            //telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            //telemetry.addData("Current Tower Height: ", towerHeight);
            telemetry.update();
            idle();
        }
    }
}