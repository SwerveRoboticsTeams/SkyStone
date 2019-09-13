package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="OpenCVFieldTester", group = "Test")
/*
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class OpenCVFieldTester extends MasterAutonomous
{
    //Declare variables here
    @Override
    public void runOpMode() throws InterruptedException
    {
        robotAngle = 0;
        robotX = 0;
        robotY = 0;
        initAuto();

        headingOffset = 0;
        //dankUnderglow(-0.5);
        waitForStart();
        openCVInit();
        sleep(1500);
        GoldLocation position = detectMineral();
        telemetry.addData("Position", position);
        telemetry.update();
        openCVDisable();
        while (opModeIsActive())
        {
        }
    }
}
