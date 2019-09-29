package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoCompetition")
public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        while(opModeIsActive())
        {
            vuforiaFollowObject();

            telemetry.update();
            idle();
        }
    }
}
