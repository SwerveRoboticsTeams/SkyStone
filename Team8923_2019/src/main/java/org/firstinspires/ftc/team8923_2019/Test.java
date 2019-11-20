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
            moveAuto(25.4*30,0,.1,0,3);



            // turn power counter clock
        }

    }
}
