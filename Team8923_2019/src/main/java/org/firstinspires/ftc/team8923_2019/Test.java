package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Test", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class Test extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //configureAutonomous();
        initAuto();
        telemetry.clear();
        telemetry.update();
//        double refereneAngle = imu.getAngularOrientation().secondAngle;


        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            ///moveAuto(-700,0,.1,0.1,3);
           // wait(2000);
            double delatX = 300;
            double deltaY = 400;
            telemetry.addData("Imu angle: " ,imu.getAngularOrientation().firstAngle);
            telemetry.addData("distance test: ", calculateDistance(delatX, deltaY));
            telemetry.update();

        }

    }
}
