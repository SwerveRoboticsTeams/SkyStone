package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="MoveForwardPark", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutoMoveForwardPark extends MasterAutonomous
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

            break;
        }

    }
}
