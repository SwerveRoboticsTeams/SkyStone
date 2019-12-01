package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Auto", group = "Swerve")
// @Disabled

public class AutoStoneFoundation extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        double time = 0.0;
        int allianceSide = 1;

        while (!opModeIsActive()) {
            // gamepad 2 controls movement

            if (gamepad2.b) {
                allianceSide *= -1;
                sleep(100);

            }

            telemetry.addData("Alliance Side:", allianceSide);
            telemetry.update();
        }

        // gamepad 1 controls delay
        if (gamepad1.dpad_up) {
            time += 1000;
            sleep(100);
        }
        if (gamepad1.dpad_down) {
            time -= 1000;
            sleep(100);

            waitForStart();

            // delay
            sleep((long) time);

            //arm going up
            arm1.setPower(-0.2);
            arm2.setPower(0.2);
            sleep(2000);
            arm1.setPower(0);
            arm2.setPower(0);

            //go to third block
            moveMaintainHeading(200, -700, 0, 0.6, 0.9, 3.0);
            moveMaintainHeading(0, -150, 0, 0.4, 0.6, 3.0);
            moveMaintainHeading(0, 0, -85, 0.2, 0.8, 3.0);

            // lower arm
            arm1.setPower(0.2);
            arm2.setPower(-0.2);
            pause(800);
            // extend arm
            core2.setPower(0.9);
            pause(400);
            // grab block
            mainWristServo.setPosition(.80);
            smallGrabber.setPosition(0);
            pause(550);
            core2.setPower(0);
            pause(125);
            arm1.setPower(0);
            arm2.setPower(0);
            pause(1000);
            smallGrabber.setPosition(0.5);


            moveMaintainHeading(0, 300, -85, 0.6, 0.9, 3.0);

            // raise
            arm1.setPower(-0.2);
            arm2.setPower(0.2);
            sleep(500);
            arm1.setPower(0);
            arm2.setPower(0);

            // move towards wall
            moveMaintainHeading(-800, 0, -85, 0.6, 0.9, 3.0);
            moveMaintainHeading(0, 5500, -85, 0.6, 0.9, 3.0);

            // raise arm
            arm1.setPower(-0.2);
            arm2.setPower(0.2);
            sleep(1500);
            arm1.setPower(0);
            arm2.setPower(0);

            // move towards foundation
            moveMaintainHeading(600, 50, 0, 0.6, 0.9, 3.0);
            pause(500);
            moveMaintainHeading(0, -650, 0, 0.6, 0.9, 3.0);

            leftFoundationPuller.setPosition(0.65);
            rightFoundationPuller.setPosition(0.35);
            pause(500);
            moveMaintainHeading(0, 850, 0, 0.7, 0.9, 3.0);
            leftFoundationPuller.setPosition(1);
            rightFoundationPuller.setPosition(0);


        }
    }
}
