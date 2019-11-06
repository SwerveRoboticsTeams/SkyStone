package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Park Foundation Blue", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class ParkFoundationBlue extends MasterAutonomous
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
            moveAuto(0,-100,1.0,.1,3);
            moveAuto(-25.4*15, 0, .5, .1, 3);
            break;
        }

    }
}
