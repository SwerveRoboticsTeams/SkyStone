package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name="Test", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
public class Test extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //configureAutonomous();
        //initHardware();
        initAuto();
        telemetry.clear();
        telemetry.update();



        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            autoReverseDrive = true;
            //imuPivot(imu.getAngularOrientation().firstAngle,-90,.3,1,3);
            moveLift(-500);
            break;
            //servoFoundationRight.setPosition(1.0);

            // turn power counter clock
        }

    }
}
