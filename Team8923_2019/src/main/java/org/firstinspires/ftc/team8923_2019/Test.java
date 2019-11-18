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



        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {
            reverseDrive = false;
            moveAuto(0,25.4*30,.1,0,3);

            //driveMecanum(,0,.2);

            // turn power counter clock
        }

    }
}
