package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Modular", group = "Swerve")
// @Disabled

public class AutoModular extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        // number of millimeters per tile, have to test
        double tile = 450.0;
        double tileSide = 550.0;
        int allianceSide = 1;
        // set positions of field elements and robot position
        double quarryX = 1;
        double foundationX = 1;
        double foundationY = 3;
        double bridgeY = 0;
        double robotPosX = 0;
        double robotPosY = 0;
        // set booleans for navigating to different field elements
        boolean navBridge = false;
        boolean navQuarry = false;
        boolean pullFoundation = false;
        boolean parkInside = true;

        while (!opModeIsActive()) {
            // press b on gamepad 2 to change alliance side
            if (gamepad2.y) {
                allianceSide *= -1;
                quarryX *= allianceSide;
                foundationX *= allianceSide;

                sleep(100);

            }

            // get input for actions
            if (gamepad2.a) {
                navQuarry = !navQuarry;
                sleep(100);
            }
            if (gamepad2.b) {
                pullFoundation = !pullFoundation;
                sleep(100);
            }
            if (gamepad2.x) {
                navBridge = !navBridge;
                sleep(100);
            }

            // get robot starting position
            if (gamepad2.dpad_up){
                robotPosY ++;
                sleep(100);
            }
            if (gamepad2.dpad_down) {
                robotPosY --;
                sleep(100);
            }
            if (gamepad2.dpad_right) {
                robotPosX ++;
                sleep(200);
            }
            if (gamepad2.dpad_left) {
                robotPosX --;
            }

            // get input for parking
            if (gamepad2.right_stick_y > 0) {
                parkInside = !parkInside;
                sleep(100);
            }

            // telemetry
            telemetry.addData("allianceSide 'Y' (1 is red, -1 is blue)", allianceSide);
            telemetry.addData("Pull foundation 'B'", pullFoundation);
            telemetry.addData("Navigate to quarry 'A'", navQuarry);
            telemetry.addData("Park 'X'", navBridge);
            telemetry.addData("Park inside 'Right Stick'", parkInside);
            telemetry.addData("Robot X Position (Dpad)", robotPosX);
            telemetry.addData("Robot Y Position", robotPosY);
            telemetry.update();

        }

        waitForStart();

        leftFoundationPuller.setPosition(1);
        rightFoundationPuller.setPosition(0);


        if (navQuarry) {
            /*if (pullFoundation) {
                moveMaintainHeading(1375, 0, 0, 0.4, 0.8, 3.0);
                moveMaintainHeading(1375, 0, 0, 0.4, 0.8, 3.0);
            }
            else {
                moveMaintainHeading((robotPosY - -2) * tileSide, 0, 0, 0.4, 0.8, 3.0);
            }

             */
            // move to quarry depending on robot position
            move(0, 100, 0.2, 0.4, 3.0);
            moveMaintainHeading(allianceSide * (robotPosY - -2) * tileSide, 0, 0, 0.4, 0.8, 3.0);

            while (arm1.getCurrentPosition() > -50) {
                arm1.setPower(-0.2);
                arm2.setPower(0.2);
            }
            arm1.setPower(0);
            arm2.setPower(0);

            while (core2.getCurrentPosition() < 400) {
                core2.setPower(0.9);
            }
            core2.setPower(0);
            mainWristServo.setPosition(0.25);
            smallGrabber.setPosition(0);
            // move to block
            moveMaintainHeading(260, 0, 0,  0.1, 0.5, 3.0);
            // push block forwards
            move(0, 350, 0.1, 0.4, 3.0);
            pause(1000);
            // lower arm
            while (arm1.getCurrentPosition() < -20) {
                arm1.setPower(0.2);
                arm2.setPower(-0.2);
            }
            arm1.setPower(0);
            arm2.setPower(0);
            // grab block
            smallGrabber.setPosition(1);
            pause(1000);
            moveMaintainHeading(0, 480, 0, 0.4,0.8, 3.0);
            moveMaintainHeading(-260, 0, 0, 0.4, 0.8, 3.0);

        }

        if (pullFoundation) {

            // navigate to foundation
            moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide, 0, 0, 0.4, 0.8, 3.0); //(robotPosY - foundationY) * tileSide
            moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide, 0, 0, 0.4, 0.8, 3.0);
            robotPosY += foundationY - robotPosY;
            moveMaintainHeading(0, -650, 0, 0.4, 0.8, 3.0);
            // lower foundation pullers
            leftFoundationPuller.setPosition(0.65);
            rightFoundationPuller.setPosition(0.35);
            pause(500);
            // pull foundation
            moveMaintainHeading(0, 650, 0, 0.4, 0.8, 3.0);
            // raise foundation pullers
            leftFoundationPuller.setPosition(1);
            rightFoundationPuller.setPosition(0);

        }



        if (navBridge) {
            moveMaintainHeading((bridgeY - robotPosY) * tile, 0, 0, 0.4, 0.8, 3.0);
            if (parkInside) {
                moveMaintainHeading(0, (1.75 * allianceSide - robotPosX) * tile, 0, 0.4, 0.8,3.0);
            }
            else {
                moveMaintainHeading(0, ((3 * allianceSide) - robotPosX) * tile, 0, 0.4, 0.8, 3.0);
            }

        }



    }

}
