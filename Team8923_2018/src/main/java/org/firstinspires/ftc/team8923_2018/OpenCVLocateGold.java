package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Rect;

import java.util.Locale;


@Autonomous(name="OpenCVLocateGold", group = "Swerve")
@Disabled
/**
 * Runable shell for Master Autonomous code
 */
public class OpenCVLocateGold extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        openCVInit();

        waitForStart();

        while (opModeIsActive())
        {
            openCVLocateGold();

            telemetry.update();
            idle();
        }

        openCVDisable();

    }
}
