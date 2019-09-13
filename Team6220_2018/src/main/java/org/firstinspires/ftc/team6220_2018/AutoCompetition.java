package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *  This program is an all-in-one autonomous.  It uses gamepad input prior to initialization to
 *  offer different routine options.
 */

@Autonomous(name = "AutoCompetition", group = "Autonomous")
//@Disabled

public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        runSetup();

        waitForStart();
        // Wait to start the match for 0-10 seconds, depending on setup input.
        pauseWhileUpdating(matchDelay);

        // Crater-------------------------------------------------------------------------------
        if (isCraterStart)
        {
            dropRobotAndUnlatch();

            // Detect minerals
            identifyGold();

            // Drive forward and knock off correct mineral
            knockGold(goldLocation);

            // Drive backward a small amount
            driveToPosition(0,-Constants.MINERAL_BACKWARD,1.0);

            // Turn 90 deg ccw
            turnTo(90,1.0);

            // Drive to crater, then drop off marker twice in case it gets stuck.
            driveToDepotDropMarker();
            pauseWhileUpdating(0.4);
            dropOffMarker();

            // Park in either our alliance or opponents' crater, depending on setup input.
            if(isAllianceCraterFinal)
            {
                knockPartnerMineral();
                turnTo(135,1.0);

                if(!knockPartnerMineral)
                {
                    // Drive backward, align with wall, and drive rest of way into crater.
                    driveToPosition(0, -400, 1.0);
                    moveRobot(0.0,0.6,1.0);
                    driveToPosition(0, -1250, 1.0);
                }
                else
                {
                    if((goldLocation == sampleFieldLocations.right))
                    {
                        moveRobot(0.0,0.8,1.0);
                    }
                    else if ((goldLocation == sampleFieldLocations.left))
                    {
                        moveRobot(0.0, 0.8, 2.0);
                    }
                    else if ((goldLocation == sampleFieldLocations.center))
                    {
                        moveRobot(0.0, 0.8, 1.5);
                    }
                    //moveRobot(0.0,0.6,1.5);
                    driveToPosition(0, -1300 - craterShift, 1.0);
                }
                //driveToPosition(-30, 0,1.0);
            }
            //opposing crater
            else
            {
                knockPartnerMineral();
                turnTo(45, 1.0);

                if(!knockPartnerMineral)
                {
                    // Drive backward, align with wall, and drive rest of way into crater.
                    driveToPosition(0, -400, 1.0);
                    moveRobot(180.0, 0.6, 1.0);
                    driveToPosition(0, -1250, 1.0);
                }
                else
                {
                    if((goldLocation == sampleFieldLocations.right))
                    {
                        moveRobot(180.0,0.6,1.8);
                    }
                    else if ((goldLocation == sampleFieldLocations.left))
                    {
                        moveRobot(180.0, 0.6, 0.7);
                    }
                    else if ((goldLocation == sampleFieldLocations.center))
                    {
                        moveRobot(180.0, 0.6, 1.2);
                    }
                    //moveRobot(180.0, 0.6, 1.5);
                    driveToPosition(0, -1250 - craterShift, 1.0);
                }
                //driveToPosition(30, 0,1.0);
            }

            motorArmLeft.setTargetPosition(Constants.ARM_GROUND_AUTO);
            motorArmRight.setTargetPosition(-Constants.ARM_GROUND_AUTO);
            powerArm(0.2);
            pauseWhileUpdating(1.2);
            motorArmLeft.setTargetPosition(Constants.ARM_FLIP_AUTO);
            motorArmRight.setTargetPosition(-Constants.ARM_FLIP_AUTO);
            powerArm(0.2);
            pauseWhileUpdating(0.5);
            /*while(opModeIsActive())
            {
                motorCollector.setPower(Constants.MOTOR_COLLECTOR_IN);
                idle();
            }*/
        }
        // Depot--------------------------------------------------------------------------------
        else
        {
            dropRobotAndUnlatch();

            // Detect minerals
            identifyGold();

            // Drive forward and knock off correct mineral
            knockGold(goldLocation);

            // Turn toward depot and drive into it; change turn angle based on the location of the gold mineral.
            turnTo(turnShift, 1.0);
            driveToPosition(0,890 ,1.0);

            // Park in either our alliance's or opponents' crater, depending on setup input.
            if(isAllianceCraterFinal)
            {
                //turnTo(0, 1.0);
                dropOffMarker();
                driveToPosition(0, 20, 1.0);
                dropOffMarker();
                turnTo(45,1.0);

                if(!knockPartnerMineral)
                {
                    // Drive backward, align with wall, and drive rest of way into crater.
                    driveToPosition(0, -400, 1.0);
                    if((goldLocation == sampleFieldLocations.right))
                    {
                        moveRobot(0.0,0.6,0.7);
                    }
                    else if ((goldLocation == sampleFieldLocations.left))
                    {
                        moveRobot(0.0, 0.6, 1.5);
                    }
                    else if ((goldLocation == sampleFieldLocations.center))
                    {
                        moveRobot(0.0, 0.6, 1.0);
                    }
                    //moveRobot(0.0,0.6,1.0);
                    driveToPosition(0, -1500, 1.0);
                }
                else
                {
                    // Drive backward, align with wall, and drive rest of way into crater.
                    driveToPosition(0, -400, 1.0);
                    //driveToPosition(craterShift, 0, 1.0);
                    if((goldLocation == sampleFieldLocations.right))
                    {
                        moveRobot(0.0,0.6,0.2);
                        driveToPosition(0,-600,1.0);
                    }
                    else if ((goldLocation == sampleFieldLocations.left))
                    {
                        moveRobot(0.0, 0.6, 1.2);
                        driveToPosition(0,-900,1.0);
                    }
                    else if ((goldLocation == sampleFieldLocations.center))
                    {
                        moveRobot(0.0, 0.6, 0.7);
                        driveToPosition(0,-850,1.0);
                    }
                    //driveToPosition(-950, -650, 1.0);
                    driveToPosition(-50, -50, 1.0);
                    turnTo(90,1.0);
                    driveToPosition(-1000, 0, 1.0);
                    knockPartnerMineral();
                    turnTo(90, 1.0);
                    driveToPosition(0, 150,1.0);
                }
                //driveToPosition(-30, 0,1.0);
            }
            else
            {
                dropOffMarker();
                turnTo(-45, 1.0);
                dropOffMarker();

                // Drive backward, align with wall, and drive rest of way into crater.
                driveToPosition(0, -400, 1.0);
                if((goldLocation == sampleFieldLocations.right))
                {
                    moveRobot(180.0,0.6,1.1);
                    driveToPosition(0, -1200, 1.0);
                }
                if((goldLocation == sampleFieldLocations.left))
                {
                    moveRobot(180.0,0.6,0.6);
                    driveToPosition(0, -1000, 1.0);
                }
                if((goldLocation == sampleFieldLocations.center))
                {
                    moveRobot(180.0,0.6,1.0);
                    driveToPosition(0, -1100, 1.0);
                }
                //driveToPosition(30, 0,0.8);
            }

            motorArmLeft.setTargetPosition(Constants.ARM_GROUND_AUTO);
            motorArmRight.setTargetPosition(-Constants.ARM_GROUND_AUTO);
            powerArm(0.2);
            pauseWhileUpdating(1.2);
            motorArmLeft.setTargetPosition(Constants.ARM_FLIP_AUTO);
            motorArmRight.setTargetPosition(-Constants.ARM_FLIP_AUTO);
            powerArm(0.2);
            pauseWhileUpdating(0.5);
            /*while(opModeIsActive())
            {
                motorCollector.setPower(Constants.MOTOR_COLLECTOR_IN);
                idle();
            }*/
        }
        // -------------------------------------------------------------------------------------

        // Stop the vision system.
        OpenCVVision.disable();
    }

    // Functions that encapsulate lengthy, frequently used autonomous code.-------------------------
    private void dropRobotAndUnlatch() throws InterruptedException
    {
        // Drop robot to ground.
        motorHanger.setTargetPosition(Constants.HANG_UNLATCH_POSITION);
        motorHanger.setPower(1.0);
        pauseWhileUpdating(0.3);
        servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
        motorHanger.setPower(0);
        pauseWhileUpdating(0.5);

        // Unlatch from hook while on ground, drive sideways, retract hanger, and return to original
        // position.
        motorHanger.setTargetPosition(Constants.HANG_GROUND_UNLATCH);
        motorHanger.setPower(1.0);
        pauseWhileUpdating(0.5);

        turnTo(15,1.0);
        pauseWhileUpdating(0.5);
        motorHanger.setTargetPosition(0);
        motorHanger.setPower(1.0);
        turnTo(0,1.0);

        /*
        driveToPosition(65,25,1.0);

        motorHanger.setTargetPosition(0);
        motorHanger.setPower(1.0);
        driveToPosition(-65,0,1.0);
        */

        // Raise arm out of camera's line of sight.
        motorArmLeft.setTargetPosition(Constants.ARM_TOP_AUTO);
        motorArmRight.setTargetPosition(-Constants.ARM_TOP_AUTO);
        powerArm(Constants.HIGH_ARM_POWER);
    }

    // Allows us to score alliance partner's mineral.
    private void knockPartnerMineral() throws InterruptedException
    {
        // Only do this if we have pressed the proper button in autonomous.
        if(knockPartnerMineral)
        {
            if (goldLocation == sampleFieldLocations.right)
            {
                if(isCraterStart)
                {
                    turnTo(130, 1.0);
                    // Drive forward to hit mineral and return to original position.
                    driveToPosition(0, -750, 1.0);
                }
                else
                {
                    //turnTo(60, 1.0);
                    driveToPosition(-400, 0, 0.5);
                    // Drive forward to hit mineral and return to original position.
                    driveToPosition(0, -500, 0.5);
                }

                if(!isAllianceCraterFinal && isCraterStart)
                {
                    turnTo(130, 1.0);
                    driveToPosition(0, 350, 1.0);
                }
                else if (isAllianceCraterFinal && isCraterStart)
                {
                    turnTo(130, 1.0);
                    driveToPosition(0, 250, 1.0);
                }
            }
            else if (goldLocation == sampleFieldLocations.left)
            {
                if(isCraterStart)
                {
                    turnTo(50, 1.0);
                    // Drive forward to hit mineral and return to original position.
                    driveToPosition(0, -800, 1.0);
                }
                else
                {
                    //turnTo(125, 1.0);
                    driveToPosition(400, 0, 1.0);
                    // Drive forward to hit mineral and return to original position.
                    driveToPosition(0, -450, 1.0);
                }

                if(!isAllianceCraterFinal && isCraterStart)
                {
                    turnTo(50, 1.0);
                    driveToPosition(0, 350, 1.0);
                }
                else if (isAllianceCraterFinal && isCraterStart)
                {
                    turnTo(50, 1.0);
                    driveToPosition(0, 250, 1.0);
                }
            }
            else if (goldLocation == sampleFieldLocations.center)
            {
                if(isCraterStart)
                {
                    // Drive forward to hit mineral and return to original position.
                    driveToPosition(0, -750, 1.0);
                }
                else
                {
                    // Drive forward to hit mineral and return to original position.
                    driveToPosition(0, -400, 1.0);
                }

                if(!isAllianceCraterFinal && isCraterStart)
                {
                    driveToPosition(0, 350, 1.0);
                }
                else if (isAllianceCraterFinal && isCraterStart)
                {
                    driveToPosition(0, 250, 1.0);
                }
            }
        }
    }

    // Code for spitting team marker out of collector.
    private void dropOffMarker() throws InterruptedException
    {
        servoMarker.setPosition(Constants.SERVO_MARKER_DEPLOYED);
        pauseWhileUpdating(0.4);
        servoMarker.setPosition(Constants.SERVO_MARKER_RETRACTED);
    }

    // Gives us the option to knock off our alliance partner's mineral if it is in the
    // right position, which is near the path of our robot anyway.
    private void driveToDepotDropMarker() throws InterruptedException
    {
        // Drive forward; we change this value based on where the gold mineral was.
        driveToPosition(0, 1250 + mineralShift, 1.0);
        turnTo(135,1.0);
        // Align robot with wall.
        moveRobot(0.0,0.6,0.8);
        // Drive forward.
        driveToPosition(0,1150,1.0);

        // Move robot away from wall.
        driveToPosition(-230,0,1.0);
        // Turn robot toward corner and drop marker.
        turnTo(90,1.0);
        dropOffMarker();
    }
    //----------------------------------------------------------------------------------------------
}
