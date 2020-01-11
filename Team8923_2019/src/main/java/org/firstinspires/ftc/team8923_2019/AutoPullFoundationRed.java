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
        initAuto();
        telemetry.clear();
        telemetry.update();

        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            autoReverseDrive = true;
            imuMoveAuto(11 ,11 ,1,.2,3);
            imuMoveAuto(0 ,23 ,1,.2,3);
            grabbersDown();
            sleep(250);
            imuMoveAuto(0,-32,1,.2,3);
            grabbersUp();
            sleep(250);
            imuMoveAuto(-55,0,1,.2,3);

            break;
        }

    }
}
