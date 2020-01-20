package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Park Bridge Delay Red", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutoParkBridgeDelayRed extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //configureAutonomous();
        enterDelay();
        initAuto();
        autoReverseDrive = true;
        telemetry.clear();
        telemetry.update();


        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            imuMoveAuto(0, 9, .5, .1, 3);
            sleep(numOfSecondsDelay);
            imuMoveAuto(-28,0,.7,.3,3);
            break;
        }

    }
}
