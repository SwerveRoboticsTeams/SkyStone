package org.firstinspires.ftc.team8923_2019;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

abstract class MasterAutonomous<rotationFilter, robotAngle> extends Master
{
    private ElapsedTime runtime = new ElapsedTime();
    // robot's position and angle on the field tracked in these variables
    double robotX;
    double robotY;
    double robotAngle;
    double headingOffset = 0.0;

    PIDFilter translationFilter;
    PIDFilter rotationFilter;

    double Kmove = 1.0f / 1200.0f;

    int newTargetFL;
    int newTargetFR;
    int newTargetBL;
    int newTargetBR;

    double speedFL;
    double speedFR;
    double speedBL;
    double speedBR;

    int errorFL;
    int errorFR;
    int errorBL;
    int errorBR;

    // Used to calculate distance traveled between loops
    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;




    Alliance alliance = Alliance.BLUE;
    StartLocations startLocation = StartLocations.DEPOT_SIDE;
    Objectives objective = Objectives.PARK;




    boolean doneSettingUp = false;

    int delays = 0;
    //int numDelays = 0;
    int delayTime = 0;

    // these values equal to one over the value (in mm for drive power and degrees for turn power)
    // that you want the PID loop to start limiting the speed at
    //Have to put double so it divides correctly
    double DRIVE_POWER_CONSTANT = 1.0 / 1000; // start slowing down at 1 meter from the target location
    double TURN_POWER_CONSTANT = 1.0 / 65; // start slowing down at 35 degrees away from the target angle;

    double MIN_DRIVE_POWER = 0.2; // don't let the robot go slower than this speed
    int TOL = 100;




    Orientation lastAngles = new Orientation();

    double globalAngle, power = .30, correction;




    enum Alliance
    {
        BLUE,
        RED
    }

    enum StartLocations
    {
        DEPOT_SIDE,
        BUILD_SIDE

    }

    enum Objectives
    {
        PULL_FOUNDATION_AND_PARK,
        PARK,
    }

    public void configureAutonomous()
    {

        // waste the zero index because we can't have zero delays
        boolean doneSettingUp = false;
        while (!doneSettingUp)
        {
            if (gamepad1.x)
                alliance = Alliance.BLUE;
                //means we are blue alliance
            else if (gamepad1.b)
                alliance = Alliance.RED;


            if(gamepad1.dpad_left)
                startLocation = StartLocations.DEPOT_SIDE;


            if (gamepad1.dpad_right)
                startLocation = StartLocations.BUILD_SIDE;
                reverseDrive = true;

            if (gamepad1.dpad_up)
                delays++;

            if(gamepad1.dpad_down && delays >= 1)
                delays--;

            if(gamepad1.a){
                switch (objective){
                    case PARK:
                        objective = Objectives.PULL_FOUNDATION_AND_PARK;
                        break;
                    case PULL_FOUNDATION_AND_PARK:
                        objective = Objectives.PARK;
                        break;
                }

            }

            if (gamepad1.start)
                doneSettingUp = true;


            while (!buttonsAreReleased(gamepad1))
            {
                telemetry.update();
                idle();
            }

//            // input information
            telemetry.addLine("Alliance Blue/Red: X/B");
            telemetry.addLine("Starting Position Depot/BuildSide: D-Pad Left/Right");
            telemetry.addLine("Add a delay: D-Pad Up/Down");
            telemetry.addLine("toggle objective: a park/park and foundation");
            telemetry.addLine("After routine is complete and robot is on field, press Start");

            telemetry.addLine();
            telemetry.addData("alliance: ", alliance);
            telemetry.addData( "startLocation:", startLocation);
            telemetry.addData("delays:", delays);
            telemetry.addData("objective", objective);
            telemetry.update();
        }
    }

    public void initAuto()
    {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initHardware();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Makes sure imu is calibrated before continuing
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Init State", "Init Finished");
        telemetry.addData("Alliance", alliance.name());
        telemetry.addData("Side", startLocation.name());
        telemetry.addData("Delay Time", delayTime);
        telemetry.update();
        // init Filters
        rotationFilter = new PIDFilter(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);
        translationFilter = new PIDFilter(Constants.TRANSLATION_P, Constants.TRANSLATION_I, Constants.TRANSLATION_D);

        // Set last known encoder values
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        // Set IMU heading offset
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        grabbersUp();
        telemetry.clear();
        telemetry.update();
        telemetry.addLine("Initialized. Ready to start!");

    }

    void dankUnderglow(double power)
    {
        motorDankUnderglow.setPower(power);
    }

    void fastFlex()
    {
        dankUnderglow(0.0);
        sleep(100);
        dankUnderglow(-1.0);
        sleep(100);
        dankUnderglow(0.0);
        sleep(100);
        dankUnderglow(-1.0);
        sleep(100);
        dankUnderglow(0.0);
        sleep(100);
        dankUnderglow(-1.0);
        idle();
    }


    /*
    How to use imu to drive straight
    1. declare variables
     initial x and y, delta X and Y, initial heading
    2. in loop (check if robot has reached location || angle is off with tolerances)
       1. recalculate delta x and y (use to find difference between the robot's current and end location)
        - need to do some encoder stuff
       2. recalculate heading difference
        - need to normalize angle
       3. roll and get filters of rotation and translation
       4. driveMecanum(driveAngle, drivePower, rotationPower)


    */
    public void moveAuto(double initDeltaX, double initDeltaY, double maxSpeed, double minSpeed, double timeout) throws InterruptedException
    {


        // Setting initial heading and robot angle to raw imu value
        final double initHeading =  normalizeAngle(imu.getAngularOrientation().firstAngle);
        robotAngle = imu.getAngularOrientation().firstAngle;

        // Recalculate these variables every loop--------------------

        double deltaX = initDeltaX;
        double deltaY = initDeltaY;

            // calculates heading difference (should return 0)
        double headingDiff = normalizeAngle(initHeading - robotAngle);
        double distanceToTarget = calculateDistance(deltaX, deltaY);


        // Values we're trying to calculate to put into driveMecanum every loop
        double driveAngle;
        double drivePower;
        double rotationPower;


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        newTargetFL = motorFL.getCurrentPosition() + (int) Math.round(Constants.COUNTS_PER_MM * y) + (int) Math.round(Constants.COUNTS_PER_MM * x * 1.15);
//        newTargetFR = motorFR.getCurrentPosition() + (int) Math.round(Constants.COUNTS_PER_MM * y) - (int) Math.round(Constants.COUNTS_PER_MM * x * 1.15);
//        newTargetBL = motorBL.getCurrentPosition() + (int) Math.round(Constants.COUNTS_PER_MM * y) - (int) Math.round(Constants.COUNTS_PER_MM * x * 1.15);
//        newTargetBR = motorBR.getCurrentPosition() + (int) Math.round(Constants.COUNTS_PER_MM * y) + (int) Math.round(Constants.COUNTS_PER_MM * x * 1.15);
//
//        if(reverseDrive == true){
//            newTargetFL = -newTargetFL;
//            newTargetFR = -newTargetFR;
//            newTargetBL = -newTargetBL;
//            newTargetBR = -newTargetBR;
//        }


        runtime.reset();

        // Check to see if robot has arrived at destination within angle and position tolerances
        // todo Check conditional
        while (((headingDiff > Constants.ANGLE_TOLERANCE_DEG) || (distanceToTarget > Constants.POSITION_TOLERANCE_MM) || (runtime.seconds() < timeout)) && !isStopRequested())
        {

            telemetry.update();
            // todo Check if this actually returns the delta x and y in mm
            // Calculate difference in X and Y based on average encoder counts of motors
            deltaX = initDeltaX - Constants.COUNTS_PER_MM * (motorFL.getCurrentPosition() -
                    motorBL.getCurrentPosition() + motorFR.getCurrentPosition() - motorBR.getCurrentPosition()) / 4;
            deltaY = initDeltaY - Constants.COUNTS_PER_MM * (motorFL.getCurrentPosition() +
                    motorBL.getCurrentPosition() - motorFR.getCurrentPosition() - motorBR.getCurrentPosition()) / 4;



            // Recalculates how far off robot is form its initial heading
            robotAngle = imu.getAngularOrientation().firstAngle;
            headingDiff = normalizeAngle(initHeading - robotAngle);

            // Recalculates distance left and driveAngle
            distanceToTarget = calculateDistance(deltaX, deltaY);
            driveAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

            // Transform position and heading diffs
            translationFilter.roll(-distanceToTarget);
            drivePower = translationFilter.getFilteredValue();
            rotationFilter.roll(headingDiff);
            rotationPower = rotationFilter.getFilteredValue();

            // Ensure robot doesn't approach target position too slowly
            if (Math.abs(drivePower) < minSpeed)
            {
                drivePower = Math.signum(drivePower) * minSpeed;
            }
            // Ensure robot doesn't ever drive faster than we want it to
            else if (Math.abs(drivePower) > maxSpeed)
            {
                drivePower = Math.signum(drivePower) * maxSpeed;
            }



//            newTargetFL += correction;
//            errorFL = newTargetFL - motorFL.getCurrentPosition();
//            speedFL = Math.abs(errorFL * Kmove);
//            speedFL = Range.clip(speedFL, minSpeed, speed);
//            speedFL = speedFL * Math.signum(errorFL);
//
//            newTargetFR -= correction;
//            errorFR = newTargetFR - motorFR.getCurrentPosition();
//            speedFR = Math.abs(errorFR * Kmove);
//            speedFR = Range.clip(speedFR, minSpeed, speed);
//            speedFR = speedFR * Math.signum(errorFR);
//
//            newTargetBL += correction;
//            errorBL = newTargetBL - motorBL.getCurrentPosition();
//            speedBL = Math.abs(errorBL * Kmove);
//            speedBL = Range.clip(speedBL, minSpeed, speed);
//            speedBL = speedBL * Math.signum(errorBL);
//
//            newTargetBR -= correction;
//            errorBR = newTargetBR - motorBR.getCurrentPosition();
//            speedBR = Math.abs(errorBR * Kmove);
//            speedBR = Range.clip(speedBR, minSpeed, speed);
//            speedBR = speedBR * Math.signum(errorBR);


            reverseDrive = true;
            driveMecanum(driveAngle - 90, drivePower, rotationPower);

//            motorFL.setPower(speedFL);
//            motorFR.setPower(speedFR);
//            motorBL.setPower(speedBL);
//            motorBR.setPower(speedBR);


            telemetry.addData("driveAngle", driveAngle-90);
            telemetry.addData("drivePower", drivePower);
            telemetry.addData("rotationPower", rotationPower);
            telemetry.addData("distance left", distanceToTarget);
            telemetry.addData("distanceFilter",translationFilter.getFilteredValue());
            telemetry.addData("rotationFilter",rotationFilter.getFilteredValue());
            telemetry.update();

            idle();
        }



        stopDriving();
    }


//    private void resetAngle()
//    {
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    private double getAngle()
//    {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }
//
//    private double checkDirection()
//    {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // You will have to experiment with your robot to get small smooth direction changes
//        // to stay on a straight line.
//        double correction, angle, gain = .10;
//
//        angle = getAngle();
//
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.
//
//        correction = correction * gain;
//
//        return correction;
//    }









    // normalizing the angle to be between -180 to 180
    public double adjustAngles(double angle)
    {
        while (angle > 180)
            angle -= 360;
        while (angle < -180)
            angle += 360;
        return angle;
    }

    void updateRobotLocation()
    {
        // Update robot angle
        // subtraction here b/c imu returns a negative rotation when turned to the right
        robotAngle = headingOffset - imu.getAngularOrientation().firstAngle;

        // Calculate how far each motor has turned since last time
        int deltaFL = motorFL.getCurrentPosition() - lastEncoderFL;
        int deltaFR = motorFR.getCurrentPosition() - lastEncoderFR;
        int deltaBL = motorBL.getCurrentPosition() - lastEncoderBL;
        int deltaBR = motorBR.getCurrentPosition() - lastEncoderBR;

        // Take average of encoder ticks to find translational x and y components. FR and BL are
        // negative because of the direction at which they turn when going sideways
        double deltaX = (deltaFL - deltaFR - deltaBL + deltaBR) / 4.0;
        double deltaY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4.0;

        telemetry.addData("deltaX", deltaX);
        telemetry.addData("deltaY", deltaY);

        // Convert to mm
        //TODO: maybe wrong proportions? something about 70/30 effectiveness maybe translation to MM is wrong
        deltaX *= Constants.MM_PER_TICK;
        deltaY *= Constants.MM_PER_TICK;

        /*
         * Delta x and y are intrinsic to the robot, so they need to be converted to extrinsic.
         * Each intrinsic component has 2 extrinsic components, which are added to find the
         * total extrinsic components of displacement. The extrinsic displacement components
         * are then added to the previous position to set the new coordinates
         */

        robotX += deltaX * Math.sin(Math.toRadians(robotAngle)) + deltaY * Math.cos(Math.toRadians(robotAngle));
        robotY += deltaX * -Math.cos(Math.toRadians(robotAngle)) + deltaY * Math.sin(Math.toRadians(robotAngle));

        // Set last encoder values for next loop
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();
    }

    public void stopDriving()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }


    public void sendTelemetry()
    {
        // Inform drivers of robot location
        telemetry.addData("X", robotX);
        telemetry.addData("Y", robotY);
        telemetry.addData("Robot Angle", imu.getAngularOrientation().firstAngle);



        // Debug info
        /*telemetry.addData("FL Encoder", motorFL.getCurrentPosition());
        telemetry.addData("FR Encoder", motorFR.getCurrentPosition());
        telemetry.addData("BL Encoder", motorBL.getCurrentPosition());
        telemetry.addData("BR Encoder", motorBR.getCurrentPosition());*/

        telemetry.update();
    }

    public void grabbersDown()
    {
        servoFoundationLeft.setPosition(1.0);
        servoFoundationRight.setPosition(0.0);
    }

    public void grabbersUp()
    {
        servoFoundationLeft.setPosition(0.0);
        servoFoundationRight.setPosition(1.0);
    }







    public double normalizeAngle(double rawAngle)
    {
        while (Math.abs(rawAngle) > 180)
        {
            rawAngle -= Math.signum(rawAngle) * 360;
        }

        return rawAngle;
    }

    void imuPivot(double referenceAngle, double targetAngle, double MaxSpeed, double kAngle, double timeout)
    {
        runtime.reset();
        //counter-clockwise is positive
        double pivot;
        double currentRobotAngle;
        double angleError;

        targetAngle = referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do
        {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            targetAngle = adjustAngles(targetAngle);//Makes it so the target angle does not wrap
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            if (pivot >= 0.0)
            {
                pivot = Range.clip(pivot, 0.15, MaxSpeed);
            } else
            {
                pivot = Range.clip(pivot, -MaxSpeed, -0.15);
            }

            speedFL = pivot;
            speedFR = -pivot;
            speedBL = pivot;
            speedBR = -pivot;

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);
            idle();
        }
        while ((opModeIsActive() && (Math.abs(angleError) > 3.0)) && (runtime.seconds() < timeout));

        stopDriving();
    }

    void reverseImuPivot(double referenceAngle, double targetAngle, double MaxSpeed, double kAngle, double timeout)
    {
        runtime.reset();
        //counter-clockwise is positive
        double pivot;
        double currentRobotAngle;
        double angleError;

        targetAngle = referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do
        {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            targetAngle = adjustAngles(targetAngle);//Makes it so the target angle does not wrap
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            if (pivot >= 0.0)
            {
                pivot = Range.clip(pivot, 0.15, MaxSpeed);
            } else
            {
                pivot = Range.clip(pivot, -MaxSpeed, -0.15);
            }

            speedFL = -pivot;
            speedFR = pivot;
            speedBL = -pivot;
            speedBR = pivot;

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);
            idle();
        }
        while ((opModeIsActive() && (Math.abs(angleError) > 3.0)) && (runtime.seconds() < timeout));

        stopDriving();
    }

    void driveToPoint(double targetX, double targetY, double targetAngle) throws InterruptedException
    {
        updateRobotLocation();

        // Calculate how far we are from target point
        double distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double DISTANCE_TOLERANCE = 40; // In mm
        double ANGLE_TOLERANCE = 5; // In degrees

        // Run until robot is within tolerable distance and angle
        while (!(distanceToTarget < DISTANCE_TOLERANCE && deltaAngle < ANGLE_TOLERANCE) && opModeIsActive())
        {
            updateRobotLocation();

            // In case robot drifts to the side
            //TODO: ISSUE IN THIS CALCULATION (BEHAVIOUR: DRIVE ANGLE DRIFTS CONSISTENTLY NEGATIVE WHILE RUNNING)
            double driveAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX)) - robotAngle;

            // Decrease power as robot approaches target. Ensure it doesn't exceed power limits
            double drivePower = Range.clip(distanceToTarget * DRIVE_POWER_CONSTANT, MIN_DRIVE_POWER, 0.3);

            // In case the robot turns while driving
            deltaAngle = subtractAngles(targetAngle, robotAngle);
            double turnPower = -deltaAngle * TURN_POWER_CONSTANT;

            // Set drive motor powers
            driveMecanum(driveAngle, drivePower, turnPower);

            // Recalculate distance for next check
            distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);

            idle();

            telemetry.addData("driveAngle", driveAngle);
            telemetry.addData("drivePower", drivePower);
            telemetry.addData("turnPower", turnPower);

            sendTelemetry();
        }
        stopDriving();
    }


    void turnToAngle(double targetAngle) throws InterruptedException
    {
        updateRobotLocation();

        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double ANGLE_TOLERANCE = 5.0; // In degrees

        while (Math.abs(deltaAngle) > ANGLE_TOLERANCE && opModeIsActive())
        {
            updateRobotLocation();

            // Recalculate how far away we are
            deltaAngle = subtractAngles(targetAngle, robotAngle);

            // Slow down as we approach target
            double turnPower = Range.clip(deltaAngle * TURN_POWER_CONSTANT, -1.0, 1.0);

            // Make sure turn power doesn't go below minimum power
            if (turnPower > 0 && turnPower < MIN_DRIVE_POWER)
                turnPower = MIN_DRIVE_POWER;
            else if (turnPower < 0 && turnPower > -MIN_DRIVE_POWER)
                turnPower = -MIN_DRIVE_POWER;

            // Set drive motor power
            driveMecanum(0.0, 0.0, turnPower);
            sendTelemetry();
            idle();
        }
        stopDriving();
    }

}