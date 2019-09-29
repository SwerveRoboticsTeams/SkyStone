package org.firstinspires.ftc.team6220_2019;

public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        while(opModeIsActive()) {
            vuforiaFollowObject();

            telemetry.update();
            idle();
        }
    }
}
