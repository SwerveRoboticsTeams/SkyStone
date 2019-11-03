package org.firstinspires.ftc.team6220_2019;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

@Autonomous(name = "TestAuto")
public class TestAuto extends MasterAutonomous
{

    public int[][] navPoints = {{-18, -18},
            {-18, 18},
            {-18, 18},
            {-18, -18}};

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        setRobotStartingOrientation(0);

        vRes.activateTargets();
        waitForStart();

        /*
        // Position robot so it can view SkyStone image target
        navigateUsingEncoders(0,16,0.7, false);

        // Drive toward stone while collecting
        navigateUsingEncoders(0,28,0.3, true);*/
        // Find SkyStone image target and translate appropriate distance toward image target
        vuforiaAlignWithSkyStone();

        //int targetNum = 0;
        while (opModeIsActive())
        {
            /*vRes.getLocation();
            //vuforiaFollowObject();

            //driveToCoordinates(0, 0, 0);

            // todo Test this code in order to implement more general navigation.
            if(driveToCoordinates(navPoints[targetNum][0], navPoints[targetNum][1], 0)){
                if(targetNum < navPoints.length - 1){
                    targetNum++;
                }
                else{
                    break;
                }
            }
            */
            telemetry.update();
            idle();
        }

        vRes.deactivateTargets();
    }
}
