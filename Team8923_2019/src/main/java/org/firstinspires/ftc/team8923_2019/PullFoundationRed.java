package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Pull Foundation Red", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class PullFoundationRed extends MasterAutonomous
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
            moveAuto(-425, 0, .2, .1, 3);
            moveAuto(0, 175, .6, .1, 3);
            moveAuto(0, -750, .4, .1, 3);
            moveAuto(0, -200, .2, .1, 3);
            sleep(200);
            grabbersDown();
            sleep(500);
            moveAuto(0, 1150, .2, .1, 3);
            grabbersUp();
            sleep(300);
            moveAuto(0,200,1.0,.1,3);
            moveAuto(900, 0, .4, .1, 3);
            moveAuto(0, -100, .4, .1, 3);
            moveAuto(600, 0, .4, .1, 3);



            break;
        }

    }
}
