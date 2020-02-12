package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.team417_2019.Resources.AvgFilter;
import org.firstinspires.ftc.team417_2019.Resources.Toggle;

abstract public class MasterTeleOp extends MasterOpMode
{
    double x = 0;
    double y = 0;
    double rotationalPower = 0;

    double curRevPos = INIT_REV_POS; // starts in down position
    double autoRevPos = 0.0;
    double autoDouble = 0.0;

    final double ADAGIO_POWER = 0.45;

    boolean isReverseMode = false;
    boolean isSlowMode = false;

    boolean controlArm = false;
    boolean isExtending = false;

    Toggle grabber = new Toggle();
    Toggle puller = new Toggle();
    Toggle slowMode = new Toggle();
    Toggle reverseMode = new Toggle();


    AvgFilter filterJoyStickInput = new AvgFilter();

    
    void driveRobot()
    {
        if (slowMode.getToggle(gamepad1.right_bumper)) {
            isSlowMode = !isSlowMode;
        }
        if (reverseMode.getToggle(gamepad1.left_bumper)) {
            isReverseMode = !isReverseMode;
        }

        y = -gamepad1.right_stick_y; // Y axis is negative when up
        x = gamepad1.right_stick_x;
        rotationalPower = gamepad1.left_stick_x;

        if (isSlowMode) {
            y *= 0.2;
            x *= 0.2;
            rotationalPower *= 0.2;
        }
        else if (isReverseMode) {
            y *= -1;
            x *= -1;

        }


        // todo check and test to see if we need filtering
        /*
        filterJoyStickInput.appendInput(x, y, pivotPower);

        x = filterJoyStickInput.getFilteredX();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();
         */

        double drivePower = Math.hypot(x, y);
        double angle = Math.atan2(x,y);

        mecanumDrive(angle, drivePower, rotationalPower);
    }

    void linearSlides()
    {
        // control the extending with G2 right (extend) and left (retract) trigger
        //if (gamepad2.right_trigger != 0 && targetCorePos < MAX_CORE_POS) core2.setPower(0.7); // extend
        //if (gamepad2.left_trigger != 0 && targetCorePos > MIN_CORE_POS) core2.setPower(-0.8); // retract

        // control the extending with G2 right (extend) and left (retract) trigger
        if (gamepad2.right_trigger != 0)
        {
            double power = Range.clip(0.9 * gamepad2.right_trigger,0.3,1.0);
            core2.setPower(power);
        }
        else if (gamepad2.left_trigger != 0)
        {
            isExtending = false;
            double power = Range.clip(-0.9 * gamepad2.left_trigger,-1.0,-0.3);
            core2.setPower(power);
        }
        else if (gamepad2.left_trigger==0 && gamepad2.right_trigger==0 && !isExtending)
        {
            core2.setPower(0.0);
        }

        if (arm1.getCurrentPosition() > -790) isExtending = false;

        // Press button y to toggle up and down
        if (gamepad2.y && !isExtending)
        {
            //isYButtonPressed = true;
            isExtending = !isExtending;
        }
        //isYButtonPressed = gamepad2.y;
        if (isExtending && arm1.getCurrentPosition() < -790) core2.setPower(0.1); // extends with toggle

    }

    void foundationPullers()
    {

        if(puller.getToggle(gamepad1.a))
        {
            leftFoundationPuller.setPosition(0.65);
            rightFoundationPuller.setPosition(0.35);
        }
        else
        {
            leftFoundationPuller.setPosition(1);
            rightFoundationPuller.setPosition(0);
        }

    }

    void lower()
    {
        while(arm1.getCurrentPosition() < 0 && arm2.getCurrentPosition() > 0) {
            arm1.setPower(0.8);
            arm2.setPower(-0.8);
        }
        arm1.setPower(0);
        arm2.setPower(0);
    }

    void collector()
    {
        if (gamepad2.x){
            controlArm = true;
        }
        if (controlArm) {
            arm1.setPower(arm1.getCurrentPosition() / -200);
            arm2.setPower(arm2.getCurrentPosition() / 200);
            if( arm1.getCurrentPosition() > -120 || Math.abs(gamepad2.right_stick_y) > 0.2){
                controlArm = false;
            }
        }
        else
        {
            double value = gamepad2.right_stick_y;
            double power = 0.8 * value * value * value + 0.15 * value * value + 0.2 * value;
            arm1.setPower(power);
            arm2.setPower(-power);
        }

        if (gamepad2.dpad_down)
        {
            mainWristServo.setPosition(0.47);
        }
        else {
                // Set Automatic Rev servo position
                // took a bunch of positions for arm + wrist so part of linear equation
                autoDouble = (double) (arm1.getCurrentPosition() + 100);
                // the slope of the equation
                autoDouble *= -0.00033; //0.4 / -1200
                // when the arm is 0 we want the wrist to be 0.25
                autoRevPos =  autoDouble + 0.235; // high pos = -1058, low = 0
                mainWristServo.setPosition(autoRevPos);
        }
        // control arm motors with G2 right stick


        // Toggle grabber
        if(grabber.getToggle(gamepad2.a))
        {
            // set smallGrabber to closed position
            smallGrabber.setPosition(1);
        }
        else
        {
            // set smallGrabber to open position
            if( gamepad2.dpad_down){
                smallGrabber.setPosition(0.6);
            }
            else{
                smallGrabber.setPosition(0.5);
            }
        }


// control hanger with G2 left and right bumpers
        /*
        if (gamepad2.dpad_up)
        {
            hanger.setPower(0.99); // extend the hanger
        }
        else if (gamepad2.dpad_down)
        {
            hanger.setPower(-0.99); // retract hanger
        }
        else
        {
            hanger.setPower(0.0);
        }

        if (gamepad2.left_bumper) vex1.setPower(0.79); // spit
        else if (gamepad2.right_bumper) vex1.setPower(-0.79); // suck
        else vex1.setPower(0.0);
         */

        /*
        if (gamepad2.left_bumper)
        {
            isSuckingIn = false; // cancel sucking in
            vex1.setPower(0.79); // if you press the left bumper release minerals
        }
        if (!gamepad2.left_bumper && !isSuckingIn) vex1.setPower(0.0); // if you are not pressing the left bumper do not set power to the vex motor

        if(isSuckingIn) vex1.setPower(-0.79); // if sucking in is true then set negative power so the servo spins opposite way

        if (gamepad2.right_bumper && !isRightBumperPushed) // if the right bumper is pressed && boolean for isRightBumperPushed true( that means it is currently false)
        {
            isRightBumperPushed = true; // switch the value of right bumper pushed to true
            isSuckingIn = !isSuckingIn; // and switch sucking in's boolean to sucking out or in depending on what it is
        }
        isRightBumperPushed = gamepad2.right_bumper; // update the current state of isRightBumperPushed otherwise it will always stay the same
        */
    }


    // rev1 is the wrist ( it used to be a run to position servo but now it is a normal servo)
     /*
    void revServo()
    {
        // control rev servo with G2 left stick
        if (-gamepad2.left_stick_y > 0.1 && curRevPos > 0.0) // if the joystick is UP
        {
            isCollectingPos = false;
            curRevPos = curRevPos - REV_INCREMENT; // move the collector up
        }
        else if (-gamepad2.left_stick_y < -0.1 && curRevPos < 0.9) // if the joystick is DOWN
        {
            isCollectingPos = false;
            curRevPos = curRevPos + REV_INCREMENT; // move the collector down
        }
        rev1.setPosition(curRevPos); // set the wrist REV servo position

    }
    void autoRev()
    {
        double autoRevPosDoub = (double) arm1.getCurrentPosition();
        autoRevPos = (  autoRevPosDoub - 60.0) / Krev;
        rev1.setPosition(autoRevPos);
    }

     */

    // should be skystone picking servo
    /*void skystoneGrabber()
    {
        // Press button y to toggle up and down
        if(isMarkerDown) skystoneGrabber.setPosition(MARKER_LOW);
        else skystoneGrabber.setPosition(MARKER_HIGH);

        if (gamepad2.y && !isYButtonPressed)
        {
            isYButtonPressed = true;
            isMarkerDown = !isMarkerDown;
        }
        isYButtonPressed = gamepad2.y;
    }*/

    void updateTelemetry()
    {
        //telemetry.addData("legato: ", isSlowMode);
        telemetry.addData("reverse: ", isReverseMode);
        telemetry.addData("autoRevPos:", autoRevPos);
        //telemetry.addData("motorMode", core2.getMode());
        telemetry.addData("core2:", core2.getCurrentPosition());
        telemetry.addData("arm1:", arm1.getCurrentPosition());
        telemetry.addData("mainWristServo:", mainWristServo.getPosition());
        telemetry.update();
    }
}
