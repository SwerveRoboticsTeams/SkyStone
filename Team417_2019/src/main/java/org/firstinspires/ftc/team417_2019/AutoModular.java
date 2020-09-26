package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_2019.Resources.Constants;

@Autonomous(name="Auto Modular", group = "Swerve")
// @Disabled
// todo remove all "move" statements (they have been commented)
public class AutoModular extends MasterAutonomous {

    /*variables for movement */

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
    double robotPosX = 0;
    double crossingX = 0;
    double robotPosY = 0;

    // set booleans for navigating to different field elements
    boolean navBridge = false;
    boolean navQuarry = false;
    boolean pullFoundation = false;
    boolean parkInside = true;

    // detection
    double horizontal = 0.0;

    public void runOpMode() throws InterruptedException {

        autoInitializeRobot();
        InitializeDetection();

        while (!opModeIsActive()) {

            PickAllianceSide();

            GetInputForActions();

            SetRobotStartingPosition();

            GetParkingInput();

            CalculateHorizontalForDetection();

            GenerateTelemetry();

        }

        waitForStart();
        vuforia.stop();
        findSkystone.disable();

        foundationPullerL.setPosition(1);
        foundationPullerR.setPosition(0);

        if (navQuarry) {



        }

        if (pullFoundation) {

        }

        if (navBridge) {

        }
    }

    private void CalculateHorizontalForDetection() {
        horizontal = inchesToMM(getHorizontal(findSkystone.p1, findSkystone.p2));
        if (horizontal < 0 && horizontal > -100){
            horizontal = -150;
        }
        else if(horizontal > 0 && horizontal < 100){
            horizontal = 150;
        }

        if (horizontal > 5){
            horizontal = inchesToMM(7.3);
        }
    }

    private void GenerateTelemetry() {
        // movement telemetry
        telemetry.addLine("Make X values negative if on blue side");
        telemetry.addLine("Crossing X value is the X position where robot crosses under bridge");
        telemetry.addLine();
        telemetry.addData("Robot X Position (Dpad)", robotPosX);
        telemetry.addData("Robot Y Position", robotPosY);
        telemetry.addData("allianceSide 'Y' (1 is red, -1 is blue)", allianceSide);
        telemetry.addData("Pull foundation 'B'", pullFoundation);
        telemetry.addData("Navigate to quarry 'A'", navQuarry);
        telemetry.addData("Park 'X'", navBridge);
        telemetry.addData("Crossing X value 'Bumpers'", crossingX);
        // detection telemetry
        telemetry.addData("Threshold", findSkystone.thresh);
        telemetry.addData("Horizontal", getHorizontal(findSkystone.p1, findSkystone.p2));
        telemetry.update();
    }

    private void GetParkingInput() {
        // get input for parking and traveling under bridge
        if (gamepad2.right_bumper) {
            crossingX += 1;
            sleep(300);
        }
        if (gamepad2.left_bumper) {
            crossingX -= 1;
            sleep(300);
        }
        if (-gamepad1.right_stick_y > 0)
        {
            findSkystone.thresh++;
            sleep(100);
        }
        if (-gamepad1.right_stick_y < 0)
        {
            findSkystone.thresh--;
            sleep(100);
        }
    }

    private void SetRobotStartingPosition() {
        // set robot starting position
        if (gamepad2.dpad_up){
            robotPosY += 0.5;
            sleep(200);
        }
        if (gamepad2.dpad_down) {
            robotPosY -= 0.5;
            sleep(200);
        }
        if (gamepad2.dpad_right) {
            robotPosX ++;
            sleep(200);
        }
        if (gamepad2.dpad_left) {
            robotPosX --;
            sleep(200);
        }
    }

    private void PickAllianceSide() {
        // press b on gamepad 2 to change alliance side
        if (gamepad2.y) {
            allianceSide *= -1;
            quarryX *= -1;
            foundationX *= -1;
            sleep(100);
        }
    }

    private void GetInputForActions() {
        // get input for actions
        if (gamepad2.a) {
            navQuarry = !navQuarry;
            sleep(200);
        }
        if (gamepad2.b) {
            pullFoundation = !pullFoundation;
            sleep(200);
        }
        if (gamepad2.x) {
            navBridge = !navBridge;
            sleep(200);
        }
    }

    private double getHorizontal(double p1, double p2) {
        double middleOfTheFrame = findSkystone.width/2;
        double centerOfSkyStone = (p1 + p2)/2;
        return (centerOfSkyStone - middleOfTheFrame) / (p2 - p1) * 8;
    }

    private double inchesToMM(double inches){
        return inches * Constants.mmPerInch;
    }

    /* Unused methods for now */
    private float findRange(double imageWidth){
        float x = (float) (1/imageWidth);
        return (x - 0.0004f) / 0.0002f;

        // comment
    }
}
