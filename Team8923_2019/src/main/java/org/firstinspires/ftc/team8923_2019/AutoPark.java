package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Park", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutoPark extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //configureAutonomous();
        initAuto();
        telemetry.clear();
        telemetry.update();


        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            imuMoveAuto(0, 3, .5, .1, 3);
            break;
        }

    }
}
