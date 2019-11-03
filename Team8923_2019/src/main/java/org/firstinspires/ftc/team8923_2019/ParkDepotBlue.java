package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Park Depot Blue", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class ParkDepotBlue extends MasterAutonomous
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
            moveAuto(0, -750, .5, .1, 3);
            moveAuto(700, 0, .5, .1, 3);
            break;
        }

    }
}
