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
        double foundationY = 2.5;
        double bridgeY = 0;
        // crossingX is the X value that the robot crosses the bridge along
        double crossingX = 2;
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
                sleep(200);
            }

            // get input for parking and traveling under bridge
            if (gamepad2.right_bumper) {
                parkInside = !parkInside;
                crossingX += 1;
                sleep(100);
            }
            if (gamepad2.left_bumper) {
                crossingX -= 1;
                sleep(100);
            }

            // telemetry
            telemetry.addData("allianceSide 'Y' (1 is red, -1 is blue)", allianceSide);
            telemetry.addData("Pull foundation 'B'", pullFoundation);
            telemetry.addData("Navigate to quarry 'A'", navQuarry);
            telemetry.addData("Park 'X'", navBridge);
            telemetry.addData("Crossing X value 'Bumpers'", crossingX);
            telemetry.addData("Robot X Position (Dpad)", robotPosX);
            telemetry.addData("Robot Y Position", robotPosY);
            telemetry.update();

        }

        waitForStart();

        leftFoundationPuller.setPosition(1);
        rightFoundationPuller.setPosition(0);


        // raise arm
        while (arm1.getCurrentPosition() > -40) {
            arm1.setPower(-0.5);
            arm2.setPower(0.5);
        }
        arm1.setPower(0);
        arm2.setPower(0);
        // extend arm
        while (core2.getCurrentPosition() < 375) {
            core2.setPower(0.9);
        }
        core2.setPower(0);
        mainWristServo.setPosition(0.25);
        smallGrabber.setPosition(0.5);

        if (navQuarry) {

            // move to quarry depending on robot position
            move(0, 100, 0.4, 0.8, 3.0);
            moveMaintainHeading((robotPosY - -2) * tileSide, 0, 0, 0.4, 0.8, 3.0);


            // move to block
            moveMaintainHeading(270 * allianceSide, 0, 0,  0.1, 0.5, 3.0); // change x value for skystone
            // push block forwards
            move(0, 390, 0.1, 0.4, 3.0);
            // lower arm
            while (arm1.getCurrentPosition() < -10) {
                arm1.setPower(0.5);
                arm2.setPower(-0.5);
            }
            arm1.setPower(0);
            arm2.setPower(0);
            // grab block
            smallGrabber.setPosition(1);
            move(0, -380, 0.4,0.8, 3.0);
            // raise arm
            while (arm1.getCurrentPosition() > -50) {
                arm1.setPower(-0.7);
                arm2.setPower(0.7);
            }
            arm1.setPower(0);
            arm2.setPower(0);
            move(0, 50, 0.4, 0.8, 3.0);
            moveMaintainHeading(-270 * allianceSide, 0, 0, 0.4, 0.8, 3.0);
            // update robot position
            robotPosX = 3 * allianceSide;
            robotPosY = -2;

            // place stone on foundation if not pulling foundation
            if (!pullFoundation) {
                // de-extend arm
                while (core2.getCurrentPosition() > 150) {
                    core2.setPower(-0.9);
                }
                core2.setPower(0);

                // navigate to foundation
                moveMaintainHeading(0, -(((robotPosX - crossingX) * 400)), 0, 0.4, 0.8, 3.0); //100 + ((robotPosX - crossingX) * tile)
                moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide, 0, 0, 0.1, 0.5, 3.0); //(robotPosY - foundationY) * tileSide
                moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide, 0, 0, 0.1, 0.5, 3.0);

                // raise arm
                while (arm1.getCurrentPosition() > -200) {
                    arm1.setPower(-0.5);
                    arm2.setPower(0.5);
                }
                arm1.setPower(0);
                arm2.setPower(0);

                moveMaintainHeading(0, ((crossingX * -tile) + 520), 0, 0.2, 0.5, 3.0);
                // open grabber to drop block
                smallGrabber.setPosition(0.5);
                pause(300);

                // pull foundation
                move(0, -150,  0.3, 0.6, 3.0);


                // raise arm
                while (arm1.getCurrentPosition() > -150) {
                    arm1.setPower(-0.5);
                    arm2.setPower(0.5);
                }
                arm1.setPower(0);
                arm2.setPower(0);

                move(0, 80, 0.4, 0.8, 3.0);
                moveMaintainHeading(550 * allianceSide, 0, 0, 0.4, 0.8, 3.0);
                moveMaintainHeading(300 * allianceSide, 0, 0, 0.4, 0.8, 3.0);

                // lower arm
                while (arm1.getCurrentPosition() < -50) {
                    arm1.setPower(0.3);
                    arm2.setPower(-0.3);
                }
                arm1.setPower(0);
                arm2.setPower(0);
                // de-extend arm
                while (core2.getCurrentPosition() > 100) {
                    core2.setPower(-0.9);
                }
                core2.setPower(0);

                robotPosX = 2 * allianceSide;
                robotPosY = 1.25;
            }

        }

        if (pullFoundation) {

            // de-extend arm
            while (core2.getCurrentPosition() > 150) {
                core2.setPower(-0.9);
            }
            core2.setPower(0);

            // navigate to foundation
            moveMaintainHeading(0, -(((robotPosX - crossingX) * 425)), 0, 0.4, 0.8, 3.0); //100 + ((robotPosX - crossingX) * tile)
            // split up X movement into two smaller movements
            moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide, 0, 0, 0.1, 0.5, 3.0); //(robotPosY - foundationY) * tileSide
            moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide, 0, 0, 0.1, 0.5, 3.0);

            // raise arm
            while (arm1.getCurrentPosition() > -200) {
                arm1.setPower(-0.5);
                arm2.setPower(0.5);
            }
            arm1.setPower(0);
            arm2.setPower(0);

            moveMaintainHeading(0, ((crossingX * -tile) + 520), 0, 0.2, 0.5, 3.0);

            // lower foundation pullers
            leftFoundationPuller.setPosition(0.65);
            rightFoundationPuller.setPosition(0.35);
            pause(500);
            // pull foundation
            move(0, -950,  0.3, 0.6, 3.0);
            // raise foundation pullers
            leftFoundationPuller.setPosition(1);
            rightFoundationPuller.setPosition(0);
            // open grabber to drop block
            smallGrabber.setPosition(0.5);

            // raise arm
            while (arm1.getCurrentPosition() > -150) {
                arm1.setPower(-0.5);
                arm2.setPower(0.5);
            }
            arm1.setPower(0);
            arm2.setPower(0);

            move(0, 100, 0.4, 0.8, 3.0);
            moveMaintainHeading(550 * allianceSide, 0, 0, 0.4, 0.8, 3.0);
            // pivot 15 degrees to the right
            moveMaintainHeading(0, 0, -5 * allianceSide, 0.4, 0.8, 3.0);
            moveMaintainHeading(0, -150, 0, 0.4, 0.8, 3.0);
            moveMaintainHeading(300 * allianceSide, 0, 0, 0.4, 0.8, 3.0);

            // lower arm
            while (arm1.getCurrentPosition() < -70) {
                arm1.setPower(0.3);
                arm2.setPower(-0.3);
            }
            arm1.setPower(0);
            arm2.setPower(0);
            // de-extend arm
            while (core2.getCurrentPosition() > 100) {
                core2.setPower(-0.9);
            }
            core2.setPower(0);

            robotPosX = 3 * allianceSide;
            robotPosY = 1.25;

        }



        if (navBridge) {
            // move forward to correct position
            moveMaintainHeading(0, (crossingX - robotPosX) * tile, 0, 0.4, 0.8,3.0);
            if (Math.abs(crossingX) == 2) {
                move(0, 100, 0.4, 0.8, 3.0);
            }
            // move sideways to correct position
            moveMaintainHeading((robotPosY - bridgeY) * tile, 0, 0, 0.4, 0.8, 3.0);

        }



    }

}
