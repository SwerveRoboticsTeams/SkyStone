package org.firstinspires.ftc.team6220_2019;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

@Autonomous(name = "TestAuto")
public class TestAuto extends MasterAutonomous
{

    public int[][] navPoints = {{-36, -36},
            {-36, 36},
            {-36, -36},
            {-36, 36}};

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize(true, false);

        setRobotStartingOrientation(90);

        vRes.getLocation();

        waitForStart();

        // Vuforia encoder backup test
        driveToCoordinates(-25, -45, 90, 0.3);
        pauseWhileUpdating(1.0);
        driveToCoordinates(-35, -45, 90, 0.3);
        pauseWhileUpdating(1.0);
        driveToCoordinates(-35, -35, 90, 0.3);
        pauseWhileUpdating(1.0);
        driveToCoordinates(-35, -45, 90, 0.3);


        vRes.deactivateTargets();
    }
}
