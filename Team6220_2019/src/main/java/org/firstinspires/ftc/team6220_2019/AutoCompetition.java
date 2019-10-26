package org.firstinspires.ftc.team6220_2019;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

@Autonomous(name = "AutoCompetition")
public class AutoCompetition extends MasterAutonomous
{

    public int[][] navPoints = {{-18, -18},
                                {-18, 18},
                                {-18, 18},
                                {-18, -18}};

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        setRobotStartingOrientation(180);

        //vRes.activateTargets();
        waitForStart();

        /*// Park left or right of current position depending on alliance.
        if (isRedAlliance)
            navigateUsingEncoders(-24,0,0.5);
        else
            navigateUsingEncoders(24,0,0.5);*/

        // Park left or right of current position depending on alliance.
        if (isRedAlliance)
        {
            // Navigate to line of stones
            navigateUsingEncoders(42, 0, 0.6);

            // Start collecting
            collectorLeft.setPower(Constants.COLLECTOR_POWER);
            collectorRight.setPower(-Constants.COLLECTOR_POWER);

            // Move forward to collect stone
            navigateUsingEncoders(0, 8, 0.3);

            // Stop collecting
            collectorLeft.setPower(0);
            collectorRight.setPower(0);

            // Navigate under Skybridge
            navigateUsingEncoders(-18, 0, 0.6);
            navigateUsingEncoders(0, -30, 0.6);
        }
        else
        {
            // Navigate to line of stones
            navigateUsingEncoders(-42, 0, 0.6);

            // Start collecting
            collectorLeft.setPower(Constants.COLLECTOR_POWER);
            collectorRight.setPower(-Constants.COLLECTOR_POWER);

            // Move forward to collect stone
            navigateUsingEncoders(0, 8, 0.3);

            // Stop collecting
            collectorLeft.setPower(0);
            collectorRight.setPower(0);

            // Navigate under Skybridge
            navigateUsingEncoders(18, 0, 0.6);
            navigateUsingEncoders(0, -30, 0.6);
        }

        /*
        int targetNum = 0;
        while (opModeIsActive())
        {
            vuforiaFollowObject();
            //driveToCoordinates(0, 0, 0);

            *//*if(driveToCoordinates(navPoints[targetNum][0], navPoints[targetNum][1], 0)){
                if(targetNum < navPoints.length - 1){
                    targetNum++;
                }
                else{
                    break;
                }
            }
            //vuforiaFollowObject();*//*

            telemetry.update();
            idle();
        }

        vRes.deactivateTargets();*/
    }
}
