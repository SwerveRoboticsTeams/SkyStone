package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Pull Foundation Blue", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class PullFoundationBlue extends MasterAutonomous
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
            moveAuto(0, -40, .4, .1, 3);
            moveAuto(450, 0, .2, .1, 3);
            moveAuto(0, 80, .3, .1, 3);
            moveAuto(0, -750, .4, .1, 3);
            moveAuto(0, -160, .2, .1, 3);
            sleep(200);
            grabbersDown();
            sleep(500);
            moveAuto(0, 900, 1.0, .1, 3);
            grabbersUp();
            moveAuto(-900, 0, .4, .1, 3);
            moveAuto(0, -100, .4, .1, 3);
            moveAuto(-580, 0, .4, .1, 3);




            break;
        }

    }
}
