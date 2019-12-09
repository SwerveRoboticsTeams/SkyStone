package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Pull Foundation Blue", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutoPullFoundationBlue extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        configureAutonomous();
        initAuto();
        telemetry.clear();
        telemetry.update();


        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            moveAuto(0, -40, .5, .1, 3);
            moveAuto(300, 0, 1, .1, 3);
            moveAuto(0, 100, 1, .1, 3);
            moveAuto(0, -750, 1, .1, 3);
            moveAuto(0, -125, .4, .1, 3);
            sleep(200);
            grabbersDown();
            sleep(500);
            moveAuto(0, 1150, .4, .1, 3);
            grabbersUp();
            sleep(200);
            moveAuto(0,15,1,.1,3);
            moveAuto(-1800, 0, .8, .1, 3);


            break;
        }

    }
}
