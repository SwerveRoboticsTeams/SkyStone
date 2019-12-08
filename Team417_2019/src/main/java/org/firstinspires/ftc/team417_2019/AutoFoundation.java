package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Foundation", group = "Swerve")
// @Disabled
public class AutoFoundation extends MasterAutonomous
{
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        double xMove= 0.0;
        double yMove = 0.0;
        double time = 0.0;
        int allianceSide = 1;

        while(!opModeIsActive()){
            // gamepad 2 controls movement

            if(gamepad2.b) {
                allianceSide *= -1;
                sleep(100);

            }

            // gamepad 1 controls delay
            if (gamepad1.dpad_up)
            {
                time += 1000;
                sleep(100);
            }
            if (gamepad1.dpad_down)
            {
                time -= 1000;
                sleep(100);
            }
            telemetry.addData("X Movement(mm):", xMove);
            telemetry.addData("Y Movement(mm):", yMove);
            telemetry.addData("Time(ms):", time);
            telemetry.addData("Alliance Side:", allianceSide);
            telemetry.update();
        }

        waitForStart();

        // delay
        sleep((long)time);


        // navigate to foundation
        moveMaintainHeading(800 * allianceSide, -100*(allianceSide-1), 0, 0.5, 0.8, 3.0);
        moveMaintainHeading(0, -650, 0, 0.5, 0.8, 3.0);
        pause(500);
        leftFoundationPuller.setPosition(0.65);
        rightFoundationPuller.setPosition(0.35);
        pause(500);
        moveMaintainHeading(0, 650, 0, 0.7, 0.9, 3.0);
        leftFoundationPuller.setPosition(1);
        rightFoundationPuller.setPosition(0);
        moveMaintainHeading(-650 * allianceSide, 0, 0, 0.5, 0.8, 3.0);
        moveMaintainHeading(0, -475 + 40* (allianceSide-1), 0, 0.7, 0.9, 3.0);
        moveMaintainHeading(0, 0, -85 * allianceSide, 0.7, 0.9, 3.0);
        core2.setPower(0.9);
        pause(1250);
        core2.setPower(0);





    }
}

