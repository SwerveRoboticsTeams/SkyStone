package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Stone Foundation", group = "Swerve")
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

            // gamepad 1 controls delay
            if (gamepad1.dpad_up) {
                time += 1000;
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                time -= 1000;
                sleep(100);
            }

            telemetry.addData("Alliance Side (-1 is for red side, 1 is for blue side):", allianceSide);
            telemetry.addData("arm1:", arm1.getCurrentPosition());
            telemetry.update();
        }



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
        moveMaintainHeading(150 * allianceSide, 0, 0, 0.7, 0.9, 3.0);
        moveMaintainHeading(0, -850, 0, 0.7, 0.9, 3.0);
        moveMaintainHeading(150 * allianceSide, 125, 0, 0.7, 0.9, 3.0);
        moveMaintainHeading(0, 0, -85 * allianceSide, 0.7, 0.9, 3.0);

        // lower arm
        arm1.setPower(0.2);
        arm2.setPower(-0.2);
        pause(700);
        // extend arm
        core2.setPower(0.9);
        pause(375);
        // grab block
        mainWristServo.setPosition(0.2);
        smallGrabber.setPosition(0);
        pause(550);
        core2.setPower(0);
        pause(125);
        arm1.setPower(0);
        arm2.setPower(0);
        pause(1000);
        smallGrabber.setPosition(0.5);


        moveMaintainHeading(0, 300, -85 * allianceSide, 0.7, 0.9, 3.0);

        // raise
        arm1.setPower(-0.2);
        arm2.setPower(0.2);
        sleep(500);
        arm1.setPower(0);
        arm2.setPower(0);

        // move towards wall
        moveMaintainHeading(-800 * allianceSide, 0, -85 * allianceSide, 0.7, 0.9, 3.0);
        moveMaintainHeading(0, 4500, -85 * allianceSide, 0.7, 0.9, 3.0);
        // raise
        arm1.setPower(-0.2);
        arm2.setPower(0.2);
        sleep(700);
        arm1.setPower(0);
        arm2.setPower(0);

        // raise arm
        /*
        arm1.setPower(-0.2);
        arm2.setPower(0.2);
        sleep(800);
        arm1.setPower(0);
        arm2.setPower(0);

         */

        // move towards foundation
        moveMaintainHeading(300 * allianceSide, 50, 0, 0.7, 0.9, 3.0);
        pause(500);
        moveMaintainHeading(0, -550, 0, 0.7, 0.9, 3.0);

        // lower foundation pullers
        leftFoundationPuller.setPosition(0.65);
        rightFoundationPuller.setPosition(0.35);
        pause(200);
        // open grabber
        smallGrabber.setPosition(0.3);
        // raise arm
        arm1.setPower(-0.2);
        arm2.setPower(0.2);
        pause(400);
        arm1.setPower(0);
        arm2.setPower(0);
        // pull foundation
        moveMaintainHeading(150, 800, 0, 0.7, 0.9, 3.0);
        // raise foundation pullers
        leftFoundationPuller.setPosition(1);
        rightFoundationPuller.setPosition(0);
        moveMaintainHeading(0, -125, 0, 0.7, 0.9, 3.0);
        // move to side
        arm1.setPower(0.2);
        arm2.setPower(-0.2);
        pause(325);
        arm1.setPower(0);
        arm2.setPower(0);
        core2.setPower(-0.9);
        moveMaintainHeading(-1100 * allianceSide, 100, 0, 0.7, 0.9, 3.0);
        // lower arm to position 0
        //moveMaintainHeading(0, 0, -85 * allianceSide, 0.7, 0.9, 3.0);
        /*
        while (arm1.getCurrentPosition() > 0) {
            arm1.setPower(0.2);
            arm2.setPower(-0.2);
        }
        arm1.setPower(0);
        arm2.setPower(0);


         */





    }
}
