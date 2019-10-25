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

        setRobotStartingOrientation(90);

        vRes.activateTargets();
        waitForStart();

        //navigateUsingEncoders(400,0,0.6);
        //navigateUsingEncoders(0,100,0.6);
        //turnTo(180, 0.5);



        int targetNum = 0;
        while (opModeIsActive())
        {
            vuforiaFollowObject();
            //driveToCoordinates(0, 0, 0);

            /*if(driveToCoordinates(navPoints[targetNum][0], navPoints[targetNum][1], 0)){
                if(targetNum < navPoints.length - 1){
                    targetNum++;
                }
                else{
                    break;
                }
            }
            //vuforiaFollowObject();*/

            telemetry.update();
            idle();
        }

        vRes.deactivateTargets();
    }
}
