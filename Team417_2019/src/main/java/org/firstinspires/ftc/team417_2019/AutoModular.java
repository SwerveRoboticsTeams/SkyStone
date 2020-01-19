package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name="Auto Modular", group = "Swerve")
// @Disabled

public class AutoModular extends MasterAutonomous {

    OpenCVDetect findSkystone;
    Dogeforia vuforia;
    WebcamName webcamName;

    public double getHorizontal(double p1, double p2) {
        double middle = findSkystone.width/2;
        double p = (p1 + p2)/2;
        return (p - middle) / (p2 - p1) * 8;
    }
    public float findRange(double imageWidth){
        float x = (float) (1/imageWidth);
        return (x - 0.0004f) / 0.0002f;
    }
    public double inchesToMM(double inches){
        return inches * 25.4;
    }
    public void runOpMode() throws InterruptedException {

        autoInitializeRobot();

        // instantiate webcam
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // show display on screen
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        // specify Vuforia parameters used to instantiate engine
        parameters.vuforiaLicenseKey = "AQdgAgj/////AAABmZGzg951/0AVjcK/+QiLWG1Z1PfbTwUouhED8hlwM6qrpAncj4xoMYYOUDxF+kreiazigY0q7OMa9XeMyxNlEQvyMFdefVUGSReIxJIXYhFaru/0IzldUlb90OUO3+J4mGvnzrqYMWG1guy00D8EbCTzzl5LAAml+XJQVLbMGrym2ievOij74wabsouyLb2HOab5nxk0FycYqTWGhKmS7/h4Ddd0UtckgnHDjNrMN4jqk0Q9HeTa8rvN3aQpSUToubAmfXe6Jgzdh2zNcxbaNIfVUe/6LXEe23BC5mYkLAFz0WcGZUPs+7oVRQb7ej7jTAJGA6Nvb9QKEa9MOdn0e8edlQfSBRASxfzBU2FIGH8a";
        parameters.fillCameraMonitorViewParent = true;
        parameters.cameraName = webcamName;
        /*parameters.useExtendedTracking = false;*/
        // Instantiate the Vuforia engine
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();
        // create a new OpenCV detector and initialize
        findSkystone = new OpenCVDetect();
        findSkystone.init(hardwareMap.appContext, CameraViewDisplay.getInstance(),0 , true);
        findSkystone.setShowContours(true);
        // have vuforia enabled with the specific detector
        vuforia.setDogeCVDetector(findSkystone);
        vuforia.enableDogeCV();

        vuforia.start();

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
        double crossingX = 0;
        double robotPosX = 0;
        double robotPosY = 0;
        // set booleans for navigating to different field elements
        boolean navBridge = false;
        boolean navQuarry = false;
        boolean pullFoundation = false;
        boolean parkInside = true;
        // detection
        double horizontal = 0.0;
        double skystoneMove = 0.0;

        while (!opModeIsActive()) {
            // press b on gamepad 2 to change alliance side
            if (gamepad2.y) {
                allianceSide *= -1;
                quarryX *= -1;
                foundationX *= -1;

                sleep(100);

            }

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

            // get robot starting position
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

            // get input for parking and traveling under bridge
            if (gamepad2.right_bumper) {
                crossingX += 1;
                sleep(300);
            }
            if (gamepad2.left_bumper) {
                crossingX -= 1;
                sleep(300);
            }
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
            telemetry.addData("Skystone Position:", findSkystone.getSksytonePosition());
            telemetry.addData("Distance", findRange(findSkystone.getSksytonePosition().width));
            telemetry.addData("Horizontal", getHorizontal(findSkystone.p1, findSkystone.p2));
            telemetry.addData("Update Horizontal", horizontal );
            telemetry.update();

        }

        waitForStart();
        vuforia.stop();
        findSkystone.disable();

        leftFoundationPuller.setPosition(1);
        rightFoundationPuller.setPosition(0);


        // raise arm
        while (arm1.getCurrentPosition() > -40) {
            arm1.setPower(-0.9);
            arm2.setPower(0.9);
        }
        arm1.setPower(0);
        arm2.setPower(0);
        // extend arm
        while (core2.getCurrentPosition() < 250) {
            core2.setPower(1.0);
        }
        core2.setPower(0);
        mainWristServo.setPosition(0.25);
        smallGrabber.setPosition(0.5);

        if (navQuarry) {

            // move to quarry depending on robot position
            move(0, 100, 0.4, 1.0, 3.0);
            moveMaintainHeading((robotPosY - (-1.5)) * 600 * allianceSide, 0, 0, 0.4, 0.8, 3.0); // -2 is the quarry Y position


            // move to block
            // change 270 for translate to skystone
            moveMaintainHeading(-horizontal * allianceSide, 0, 0, 0.1, 0.9, 3.0); // CHANGE X VALUE FOR SKYSTONE, 270 is third block from wall


            // push block forwards
            move(0, 390, 0.1, 0.4, 3.0);
            // lower arm
            while (arm1.getCurrentPosition() < -20 && opModeIsActive()) {
                arm1.setPower(0.5);
                arm2.setPower(-0.5);
            }
            arm1.setPower(0);
            arm2.setPower(0);
            // grab block
            smallGrabber.setPosition(1);
            pause(100);
            move(0, -380, 0.4,0.9, 3.0);
            move(0, 50, 0.4, 0.9, 3.0);
            moveMaintainHeading(horizontal * allianceSide, 0, 0, 0.4, 0.9, 3.0);
            // update robot position
            robotPosX = 3 * allianceSide;
            robotPosY = -1.5;

            // place stone on foundation if not pulling foundation
            if (!pullFoundation) {

                // de-extend arm
                while (core2.getCurrentPosition() > 150 && opModeIsActive()) {
                    core2.setPower(-1.0);
                }
                core2.setPower(0);

                // navigate to foundation
                moveMaintainHeading(0, -(((robotPosX - crossingX) * 380) * allianceSide), 0, 0.4, 1.0, 3.0);
                moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide * allianceSide, 0, 0, 0.1, 0.6, 3.0);
                moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide * allianceSide, 0, 0, 0.1, 0.6, 3.0);

                // raise arm
                while (arm1.getCurrentPosition() > -200 && opModeIsActive()) {
                    arm1.setPower(-0.8);
                    arm2.setPower(0.8);
                }
                arm1.setPower(0);
                arm2.setPower(0);

                moveMaintainHeading(0, (((crossingX - foundationX) * -tile)), 0, 0.2, 0.9, 3.0);
                // lower arm
                while (arm1.getCurrentPosition() < -120 && opModeIsActive()) {
                    arm1.setPower(0.8);
                    arm2.setPower(-0.8);
                }
                arm1.setPower(0);
                arm2.setPower(0);
                // open grabber to drop block
                smallGrabber.setPosition(0.5);
                pause(150);

                // pull foundation
                move(0, -150,  0.3, 0.7, 3.0);


                // raise arm
                while (arm1.getCurrentPosition() > -150 && opModeIsActive()) {
                    arm1.setPower(-0.7);
                    arm2.setPower(0.7);
                }
                arm1.setPower(0);
                arm2.setPower(0);

                move(0, 80, 0.4, 0.8, 3.0);
                moveMaintainHeading(550 * allianceSide, 0, 0, 0.4, 0.9, 3.0);
                moveMaintainHeading(300 * allianceSide, 0, 0, 0.4, 0.9, 3.0);

                // lower arm
                while (arm1.getCurrentPosition() < -100 && opModeIsActive()) {
                    arm1.setPower(0.3);
                    arm2.setPower(-0.3);
                }
                arm1.setPower(0);
                arm2.setPower(0);
                // de-extend arm
                while (core2.getCurrentPosition() > 100 && opModeIsActive()) {
                    core2.setPower(-1.0);
                }
                core2.setPower(0);

                robotPosX = 2 * allianceSide;
                robotPosY = 1.25;
            }

        }

        if (pullFoundation) {

            // de-extend arm
            while (core2.getCurrentPosition() > 150 && opModeIsActive()) {
                core2.setPower(-1.0);
            }
            core2.setPower(0);

            // navigate to foundation
            // if moving on outside of field, move forward to not get stuck on wall
            if (Math.abs(crossingX) == 3) {
                move(0, 150, 0.4,0.8, 3.0);
            }
            moveMaintainHeading(0, -(((robotPosX - crossingX) * 380)* allianceSide), 0, 0.4, 1.0, 3.0);
            // split up X movement into two smaller movements
            moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide * allianceSide, 0, rotation.thirdAngle, 0.1, 0.5, 3.0);
            moveMaintainHeading(0.5 * (robotPosY - foundationY) * tileSide * allianceSide, 0, rotation.thirdAngle, 0.1, 0.5, 3.0);

            // raise arm
            while (arm1.getCurrentPosition() > -200 && opModeIsActive()) {
                arm1.setPower(-0.7);
                arm2.setPower(0.7);
            }
            arm1.setPower(0);
            arm2.setPower(0);

            // align against wall
            if (Math.abs(crossingX) == 3) {
                move(0, -200, 0.4, 0.7, 3.0);
            }
            else {
                move(0, -400, 0.4, 0.7, 3.0);
            }
            moveMaintainHeading(0, ((((3 * allianceSide) - foundationX) * -(tile - 50))* allianceSide), 0, 0.2, 0.5, 3.0);

            // lower foundation pullers
            leftFoundationPuller.setPosition(0.65);
            rightFoundationPuller.setPosition(0.35);
            pause(500);
            // pull foundation
            move(0, -900,  0.3, 0.7, 3.0);
            // raise foundation pullers
            leftFoundationPuller.setPosition(1);
            rightFoundationPuller.setPosition(0);
            // lower arm
            while (arm1.getCurrentPosition() < -120 && opModeIsActive()) {
                arm1.setPower(0.8);
                arm2.setPower(-0.8);
            }
            arm1.setPower(0);
            arm2.setPower(0);
            // open grabber to drop block
            smallGrabber.setPosition(0.5);

            // raise arm
            while (arm1.getCurrentPosition() > -170 && opModeIsActive()) {
                arm1.setPower(-0.5);
                arm2.setPower(0.5);
            }
            arm1.setPower(0);
            arm2.setPower(0);

            move(0, 100, 0.4, 0.8, 3.0);
            moveMaintainHeading(550 * allianceSide, 0, 0, 0.4, 1.0, 3.0);
            // pivot 5 degrees to the right
            //moveMaintainHeading(0, 0, -5 * allianceSide, 0.4, 1.0, 3.0);
            moveMaintainHeading(0, -150, 0, 0.4, 1.0, 3.0);
            moveMaintainHeading(300 * allianceSide, 0, 0, 0.4, 1.0, 3.0);

            // lower arm
            while (arm1.getCurrentPosition() < -100 && opModeIsActive()) {
                arm1.setPower(0.3);
                arm2.setPower(-0.3);
            }
            arm1.setPower(0);
            arm2.setPower(0);
            // de-extend arm
            while (core2.getCurrentPosition() > 100 && opModeIsActive()) {
                core2.setPower(-1.0);
            }
            core2.setPower(0);

            robotPosX = 3 * allianceSide;
            robotPosY = 1.25;

        }



        if (navBridge) {
            // move forward to correct position
            moveMaintainHeading(0, (crossingX - robotPosX) * tile * allianceSide, 0, 0.4, 1.0,3.0);
            if (Math.abs(crossingX) == 2) {
                move(0, 100, 0.4, 1.0, 3.0);
            }
            // move sideways to correct position
            moveMaintainHeading((robotPosY - bridgeY) * tile * allianceSide, 0, 0, 0.4, 1.0, 3.0);

        }
    }

}
