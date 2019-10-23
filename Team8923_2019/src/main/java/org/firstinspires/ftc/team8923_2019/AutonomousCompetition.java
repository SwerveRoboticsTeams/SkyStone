package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Autonomous Competition", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutonomousCompetition extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
       // configureAutonomous();
        telemetry.addLine("in opmode");
        telemetry.update();
        initAuto();

        waitForStart();
        //telemetry.clear();


        while (opModeIsActive())
        {
            telemetry.addLine("opMode is active");
            telemetry.update();

            //moveAuto(0, -1000,  1.0,  0.3, 3.0);


           //moveAuto(0, -1000,  1.0,  0.3, 3.0);
//            // play delay
//
//            // Figure out which start location
//            switch(startLocation)
//            {
//
//                case DEPOT_SIDE:
//                {
//
//                }
//
//                case BUILD_SIDE:
//                {
//
//                }
//
//
//            }
//
//
            // Move robot




        }

    }
}
