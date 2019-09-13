package org.firstinspires.ftc.team8923_2018;

import android.annotation.TargetApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


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
        telemetry.update();
        initAuto();

        waitForStart();
        openCVInit();
        telemetry.clear();

        while (opModeIsActive())
        {
            for (int i: delays)
            {
                sleep(i * 1000);
            }
            GoldLocation position = landAndDetectMineral();
            dankUnderglow(-1.0);
            moveAuto(0, 10, 0.2, 0.2, 3.0);//straighten out robot
            moveAuto(80, 0, 0.35, 0.2, 3.0);//move off of hook
            double referenceAngle =  imu.getAngularOrientation().firstAngle; // Get a reference ange from the IMU for future movements using IMU

            //NOTE this is only for depot side
            switch (startLocation)
            {
                case DEPOT:
                {
                    switch (position)
                    {
                        case LEFT:
                        {
                            moveAuto(0, -370, 1.0, 0.3, 3.0);
                            moveAuto(400, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, -610, 1.0, 0.3, 3.0);
                            //End of knock off mineral phase
                            //Other phases to be decided later
                            imuPivot(referenceAngle, -45, 1.0, 0.015, 3.0);
                            moveAuto(0, -420, 1.0, 0.3, 3.0);
                            moveAuto(0, 100, 1.0, 0.3, 3.0);
                            dropJJ();
                            moveAuto(0, 900, 1.0, 0.3, 3.0);
                            moveAuto(270, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, 480, 0.5, 0.5, 3.0);
                            break;

                        }
                        case CENTER:
                        {
                            moveAuto(0, -1270, 0.5, 0.35, 3.0);
                            moveAuto(0, 75, 0.7, 0.35, 3.0);
                            //End of knock off mineral phase
                            //Other phases to be decided later
                            imuPivot(referenceAngle, -90, 0.5, 0.015, 2.0);//0.5
                            moveAuto(0, 250, 0.5, 0.5, 3.0);
                            imuPivot(referenceAngle, -45, 0.5, 0.015, 2.0);//0.5
                            dropJJ();
                            moveAuto(150, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, 800, 1.0, 0.3, 3.0);
                            moveAuto(100, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, 650, 0.5, 0.5, 3.0);
                            break;
                        }
                        case RIGHT:
                        {
                            moveAuto(0, -350, 1.0, 0.3, 3.0);
                            moveAuto(-500, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, -770, 1.0, 0.5, 3.0);
                            //End of knock off mineral phase
                            //Other phases to be decided later
                            imuPivot(referenceAngle, 45, 0.7, 0.015, 3.0);
                            moveAuto(-100, 0, 0.3, 0.3, 3.0);
                            //moveAuto(0, -100, 0.3, 0.3, 3.0);
                            //dropJJ();
                            //moveAuto(100, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, -380, 0.5, 0.7, 3.0);
                            dropJJ();
                            moveAuto(0, 300, 1.0, 0.3, 3.0);
                            moveAuto(100, 0, 1.0, 0.3, 3.0);

                            imuPivot(referenceAngle, -90, 0.7, 0.015, 3.0);
                            moveAuto(0, 630, 0.5, 0.7, 3.0);
                            imuPivot(referenceAngle, -45, 0.7, 0.015, 3.0);
                            moveAuto(50, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, 800, 1.0, 0.3, 3.0);
                            moveAuto(100, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, 570, 0.5, 0.5, 3.0);
                            break;
                        }
                    }
                    break;
                }
                case CRATER:
                {
                    switch (position)
                    {
                        case LEFT:
                        {
                            moveAuto(0, -280, 0.75, 0.3, 3.0);
                            moveAuto(480, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, -340, 0.75, 0.3, 3.0);
                            //fastFlex();
                            moveAuto(0, 260, 0.75, 0.3, 3.0);
                            imuPivot(referenceAngle, 90, 0.5, 0.015, 3.0);
                            moveAuto(0, -610, 0.75, 0.3, 3.0);
                            break;
                        }
                        case CENTER:
                        {
                            moveAuto(0, -620, 0.75, 0.35, 3.0);
                            //fastFlex();
                            moveAuto(0, 260, 0.7, 0.35, 3.0);
                            imuPivot(referenceAngle, 90, 0.5, 0.015, 3.0);
                            moveAuto(0, -1010, 0.75, 0.3, 3.0);
                            break;
                        }
                        case RIGHT:
                        {
                            moveAuto(0, -380, 0.75, 0.3, 3.0);
                            moveAuto(-480, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, -340, 0.75, 0.5, 3.0);
                            //fastFlex();
                            moveAuto(0, 260, 0.75, 0.3, 3.0);
                            imuPivot(referenceAngle, 90, 0.5, 0.015, 3.0);
                            moveAuto(0, -1560, 0.75, 0.3, 3.0);
                            break;
                        }
                    }
                    imuPivot(referenceAngle, 135, 0.7, 0.015, 3.0);
                    moveAuto(-300, 0, 0.3, 0.3, 3.0);
                    moveAuto(0, -1050, 1.0, 0.7, 3.0);
                    //imuPivot(referenceAngle, 0, 0.5, 0.015, 3.0);
                    dropJJ();
                    fastFlex();
                    moveAuto(100, 0, 0.3, 0.3, 3.0);
                    moveAuto(0, 550, 1.0, 0.7, 3.0);
                    moveAuto(-250, 0, 0.3, 0.7, 3.0);
                    moveAuto(0, 1000, 1.0, 0.7, 3.0);
                    fastFlex();
                }
                break;
            }
            servoJJ2.setPosition(0.3);
            deployArmFAST();
            fastFlex();
            fastFlex();
            idle();
            break;
        }
    }
}
