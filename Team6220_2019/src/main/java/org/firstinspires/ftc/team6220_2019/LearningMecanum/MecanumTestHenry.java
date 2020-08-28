package org.firstinspires.ftc.team6220_2019.LearningMecanum;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DBTest", group = "TeleOp")
public class MecanumTestHenry extends MasterTeleOpHenry
{

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();

        telemetry.addData("Init", "Done");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            driveMecanumWithJoysticks();
        }
    }
}
