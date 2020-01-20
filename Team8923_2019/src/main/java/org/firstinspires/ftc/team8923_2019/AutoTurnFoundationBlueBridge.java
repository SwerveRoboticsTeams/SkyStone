package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Turn Foundation Blue Bridge", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutoTurnFoundationBlueBridge extends MasterAutonomous
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

            imuMoveAuto(-11 ,11 ,1,.2,3);
            imuMoveAuto(0 ,23 ,1,.2,3);
            grabbersDown();
            sleep(250);
            imuPivot(imu.getAngularOrientation().firstAngle,30,.5,1,1);
            imuMoveAuto(0 ,-23 ,1,.2,3);
            imuPivot(imu.getAngularOrientation().firstAngle,60,.5,1,1);
            imuMoveAuto(0 ,14 ,1,.2,3);
            grabbersUp();
            sleep(700);
            imuMoveAuto(8 ,0 ,1,.2,3);
            imuMoveAuto(0,-44,1,.2,3);
            break;
        }

    }
}
