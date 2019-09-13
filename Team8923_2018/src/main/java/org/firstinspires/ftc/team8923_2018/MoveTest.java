package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Move Test", group = "Test")
/*
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class MoveTest extends MasterAutonomous
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
        waitForStart();
        while (opModeIsActive())
        {
            driveToPoint(500, 500,0);
            sleep(500);
            driveToPoint(0, 0,0);
            sleep(10000);
        }
    }
}
