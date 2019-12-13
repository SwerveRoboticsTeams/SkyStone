package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Pull Foundation Red", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutoPullFoundationRed extends MasterAutonomous
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
            imuMoveAuto(0, -40, .5, .1, 3);
            imuMoveAuto(-300, 0, 1, .1, 3);
            imuMoveAuto(0, 100, 1, .1, 3);
            imuMoveAuto(0, -750, 1, .1, 3);
            imuMoveAuto(0, -125, .4, .1, 3);
            sleep(200);
            grabbersDown();
            sleep(500);
            imuMoveAuto(0, 1150, .4, .1, 3);
            grabbersUp();
            sleep(200);
            imuMoveAuto(0,15,1,.1,3);
            imuMoveAuto(1800, 0, .8, .1, 3);


            break;
        }

    }
}
