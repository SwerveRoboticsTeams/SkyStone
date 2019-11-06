package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Park Depot Red", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class ParkDepotRed extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        configureAutonomous();
        initAuto();
        telemetry.clear();
        telemetry.update();
        double refereneAngle = imu.getAngularOrientation().secondAngle;


        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            // forward
            moveAuto(0, -750, .5, .1, 3);
            // turnLeft
            imuPivot(refereneAngle,-90,.3, .015, 3);
            // forward
            moveAuto(0, 700, .5, .1, 3);
            break;
        }

    }
}
