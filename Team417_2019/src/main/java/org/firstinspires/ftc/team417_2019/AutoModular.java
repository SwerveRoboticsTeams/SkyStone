package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Modular", group = "Swerve")
// @Disabled

public class AutoModular extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        // number of millimeters per tile, have to test
        double tile = 609.6;
        int allianceSide = 1;
        // set positions of field elements and robot position
        double quarryX = 1;
        double foundationX = 1;
        double foundationY = 1.5;
        double bridgeY = 0;
        double robotPosX = 0;
        double robotPosY = 0;
        // set booleans for navigating to different field elements
        boolean navBridge = false;
        boolean navQuarry = false;
        boolean pullFoundation = false;

        while (!opModeIsActive()) {
            // press b on gamepad 2 to change alliance side
            if (gamepad2.b) {
                allianceSide *= -1;
                quarryX *= allianceSide;
                foundationX *= allianceSide;

                sleep(100);

            }

            // get input for actions
            if (gamepad2.a) {
                navQuarry = true;
            }
            if (gamepad2.b) {
                pullFoundation = true;
            }
            if (gamepad2.x) {
                navBridge = true;
            }

            // get robot starting position
            if (gamepad2.dpad_up){
                robotPosY ++;
            }
            if (gamepad2.dpad_down) {
                robotPosY --;
            }
            if (gamepad2.dpad_right) {
                robotPosX ++;
            }
            if (gamepad2.dpad_left) {
                robotPosX --;
            }

        }

        waitForStart();

        leftFoundationPuller.setPosition(1);
        rightFoundationPuller.setPosition(0);
        if (pullFoundation) {
            // navigate to foundation, need to add updating robot position
            moveMaintainHeading((foundationX - robotPosX) * tile, 0, 0, 0.4, 0.8, 3.0);
            moveMaintainHeading(0, (foundationY - robotPosY) * tile, 0, 0.4, 0.8, 3.0);

            leftFoundationPuller.setPosition(0.65);
            rightFoundationPuller.setPosition(0.35);

        }
    }
}
