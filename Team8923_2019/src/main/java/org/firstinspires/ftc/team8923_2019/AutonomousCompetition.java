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
        configureAutonomous();
        initAuto();
        telemetry.clear();
        telemetry.update();
        double referenceAngle = imu.getAngularOrientation().secondAngle;


        waitForStart();
        telemetry.clear();


        while (opModeIsActive())
        {


            // play delay
            wait(delays * 1000);


            switch(startLocation)
            {
                case DEPOT_SIDE:
                {
                    // get the skystone

                }

                case BUILD_SIDE:
                {
                    reverseDrive = true;
                    switch(objective){
                        case PARK:
                            if(alliance == Alliance.RED){
                                moveAuto(-100, 0,  1.0,  0.3, 3.0);
                            }else{
                                moveAuto(100, 0,  1.0,  0.3, 3.0);
                            }

                            break;
                        case PULL_FOUNDATION_AND_PARK:

                            if(alliance == Alliance.RED){
                                moveAuto(-100, 0,  1.0,  0.3, 3.0);
                            }else{
                                moveAuto(100, 0,  1.0,  0.3, 3.0);
                            }
                            //pull and move to foundation
                            moveAuto(0,1000,0.5,0.3,3.0);
                            // toggle servos down

                            // pull back foundation

                            // toggle servos up

                            // move to park


                            break;
                    }
                }


            }


            // Move robot




        }

    }
}
