package org.firstinspires.ftc.team417_2019;

import org.firstinspires.ftc.team417_2019.Resources.AvgFilter;
import org.firstinspires.ftc.team417_2019.Resources.Constants;
import org.firstinspires.ftc.team417_2019.Resources.Toggler;

abstract public class MasterTeleOp extends MasterOpMode
{
    private double linkagePosition = 1;
    private double timeSinceOpened = 0;

    private int liftEncoders = 0;
    private double liftInches = 0;

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;
    private boolean prevClawPosition = false;

    private Toggler slowMode = new Toggler();
    private Toggler reverseMode = new Toggler();
    private Toggler puller = new Toggler();
    private Toggler claw = new Toggler();


    AvgFilter filterJoyStickInput = new AvgFilter();

    protected MasterTeleOp() {

    }

    /**
     * Uses the mecanum drive function to move the robot | Right stick translate, Left stick rotate (gamepad1)
     */
    public void driveRobot()
    {
        isSlowMode = slowMode.toggle(gamepad1.right_bumper);
        isReverseMode = reverseMode.toggle(gamepad1.left_bumper);

        double y = -gamepad1.right_stick_y; // Y is negative above the Y axis
        double x = gamepad1.right_stick_x;
        double rotationalPower = gamepad1.left_stick_x;

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
        double angle = Math.atan2(y, x);

        mecanumDrive(angle, drivePower, rotationalPower);
    }

    /**
     * Toggle the positions of the foundation pullers | A to toggle (gamepad1)
     */
    protected void foundationPullers()
    {
        if(puller.toggle(gamepad1.a)) { //toggle the position of the foundation pullers, from all the way down, to sideways
            foundationPullerL.setPosition(1); //pointed down
            foundationPullerR.setPosition(0);
        }
        else { //starting state
            foundationPullerL.setPosition(0.5); //pointed sideways
            foundationPullerR.setPosition(0.5);
        }

    }

    public void collector()
    {

    }

    /**
     * Control the collector wheels | A to collect, Y to reverse (gamepad2)
     * @param collectionPower
     * When spinning in, motors will use this power. When spinning out, motors will use half this
     */
    protected void newCollector(double collectionPower) {
        if (gamepad2.a) { //hold down A to collect
            collectorMotorL.setPower(-collectionPower);
            collectorMotorR.setPower(-collectionPower);
        }
        else if (gamepad2.y) { //hold down Y to slowly reverse
            collectorMotorL.setPower(collectionPower);
            collectorMotorR.setPower(collectionPower);
        }
        else { //default state is idle
            collectorMotorL.setPower(0);
            collectorMotorR.setPower(0);
        }
    }

    /**
     * Control the linear slides and claw | Right stick to control slides, Left stick to control linkage, X to toggle claw (gamepad2)
     * @param liftSpeed
     * Double used to scale the lift movement speed as needed
     * @param linkageSpeed
     * Double used to scale down the linkage movement speed as needed
     */
    protected void newStacker(double liftSpeed, double linkageSpeed) {
        //right stick controls slides
        //TODO: use inches instead of encoder counts
        if (Math.abs(gamepad2.right_stick_y) > 0.1) { //deadzone
            liftInches -= gamepad2.right_stick_y * liftSpeed; //control over lift height is linearly proportional to gamepad2 right stick y
        }

        //left stick controls linkages
        if (Math.abs(gamepad2.left_stick_y) > 0.1) { //deadzone
            linkagePosition += -gamepad2.left_stick_y * linkageSpeed; //scale by a constant given in TeleOp class
        }

        if (gamepad2.right_bumper) { //set the stacker assembly to the collecting position
            liftInches = 8;
            linkagePosition = 0.7;
            claw.toggleState = false; //open the claw
        }

        if (gamepad2.left_bumper) { //set the stacker assembly to the picking up position
            liftInches = 2;
            linkagePosition = 0.67;
            claw.toggleState = false; //open the claw
        }

        linkagePosition = Math.max(Math.min(linkagePosition, 0.8), 0.2); //constrain the linkage position to min and max position
        liftInches = Math.max(Math.min(liftInches, 28.5), 0); //constrain the lift height to the bottom and top of the available movement
        liftEncoders = (int) Math.round(liftInches * Constants.encoderTicksPerRot / (2 * Constants.inchPulleyRadius * Math.PI)); //convert inches to encoder counts for an orbital 20 motor

        linkageServo.setPosition(linkagePosition); //update linkage position

        if (liftMotorR.getCurrentPosition() - liftEncoders < 0) { //true if the target encoder count is below the current slide position
            liftMotorL.setPower(0.75);
            liftMotorR.setPower(0.75);
        }
        else { //move slower if the slides are going down, to avoid a jerky motion
            liftMotorL.setPower(0.2);
            liftMotorR.setPower(0.2);
        }

        liftMotorL.setTargetPosition(-liftEncoders);
        liftMotorR.setTargetPosition(liftEncoders);

        if (claw.toggle(gamepad2.x)) { //toggle the position of the claw
            grabberServo.setPower(-1); //close
        }
        else { //open the claw
            if (prevClawPosition) { //if, on the last loop through, the claw was closed, mark the current time of the start of opening the claw
                timeSinceOpened = getRuntime();
            }

            if (getRuntime() - timeSinceOpened < 0.3) { //if it's been less than 0.3 seconds since starting to open, continue opening the claw
                grabberServo.setPower(0.5); //open
            }
            else {
                grabberServo.setPower(0); //idle the claw servo
            }
        }

        prevClawPosition = claw.getToggleState();
    }

    /**
     * Displays debugging info on the driver station
     */
    protected void updateTelemetry()
    {
        telemetry.addData("legato: ", isSlowMode);
        telemetry.addData("reverse: ", isReverseMode);
        telemetry.addData("liftInches", liftInches);
        telemetry.addData("linkagePosition", linkagePosition);
        telemetry.update();
    }
}