package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="GrabSkyStone", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class GrabSkyStoneAutonomous extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        configureAutonomous();
        initAuto();
        telemetry.clear();
        telemetry.update();
        reverseDrive = false;

        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            moveAuto(300, 0, .5, .1, 3);
            break;
        }

    }
}
