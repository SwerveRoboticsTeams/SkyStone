package org.firstinspires.ftc.team8923_2018;

import android.text.method.BaseKeyListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Locale;

abstract class MasterAutonomous extends Master
{
    private ElapsedTime runtime = new ElapsedTime();
    // robot's position and angle on the field tracked in these variables
    double robotX;
    double robotY;
    double robotAngle;
    double headingOffset = 0.0;

    double Kmove = 1.0f/1200.0f;

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
    StartLocations startLocation = StartLocations.DEPOT;
    Assist assist = Assist.NOT_ASSISTING;
    boolean doneSettingUp = false;

    ArrayList<Integer> delays = new ArrayList<>();
    int numDelays = 0;
    int delayTime = 0;

    // these values equal to one over the value (in mm for drive power and degrees for turn power)
    // that you want the PID loop to start limiting the speed at
    //Have to put double so it divides correctly
    double DRIVE_POWER_CONSTANT = 1.0/1000; // start slowing down at 1 meter from the target location
    double TURN_POWER_CONSTANT = 1.0/65; // start slowing down at 35 degrees away from the target angle;

    double MIN_DRIVE_POWER = 0.2; // don't let the robot go slower than this speed
    int TOL = 100;

    enum Alliance
    {
        BLUE,
        RED
    }
    enum StartLocations
    {
        DEPOT(0),
        CRATER(0),

        RED_DEPOT_START_X(-341.25),
        RED_DEPOT_START_Y(-341.25),
        RED_DEPOT_START_ANGLE(225.0),

        BLUE_DEPOT_START_X(341.25),
        BLUE_DEPOT_START_Y(341.25),
        BLUE_DEPOT_START_ANGLE(45.0),

        RED_CRATER_START_X(-341.25),
        RED_CRATER_START_Y(341.25),
        RED_CRATER_START_ANGLE(315.0),

        BLUE_CRATER_START_X(341.25),
        BLUE_CRATER_START_Y(341.25),
        BLUE_CRATER_START_ANGLE(135.0);

        public final double val;
        StartLocations(double i) {val = i;}
    }
    enum Assist
    {
        ASSISTING,
        NOT_ASSISTING;
    }
    enum MineralLocations
    {

        BLUE_DEPOT_LEFT_MINERAL_X(1219.20),
        BLUE_DEPOT_LEFT_MINERAL_Y(609.60),
        BLUE_DEPOT_CENTER_MINERAL_X(967.75),
        BLUE_DEPOT_CENTER_MINERAL_Y(861.01),
        BLUE_DEPOT_RIGHT_MINERAL_X(716.31),
        BLUE_DEPOT_RIGHT_MINERAL_Y(1112.50),

        BLUE_CRATER_LEFT_MINERAL_X(BLUE_DEPOT_LEFT_MINERAL_Y.val),
        BLUE_CRATER_LEFT_MINERAL_Y(-BLUE_DEPOT_LEFT_MINERAL_X.val),
        BLUE_CRATER_CENTER_MINERAL_X(BLUE_DEPOT_CENTER_MINERAL_Y.val),
        BLUE_CRATER_CENTER_MINERAL_Y(-BLUE_DEPOT_CENTER_MINERAL_X.val),
        BLUE_CRATER_RIGHT_MINERAL_X(BLUE_DEPOT_RIGHT_MINERAL_Y.val),
        BLUE_CRATER_RIGHT_MINERAL_Y(-BLUE_DEPOT_RIGHT_MINERAL_X.val),

        RED_DEPOT_LEFT_MINERAL_X(-BLUE_DEPOT_LEFT_MINERAL_X.val),
        RED_DEPOT_LEFT_MINERAL_Y(-BLUE_DEPOT_LEFT_MINERAL_Y.val),
        RED_DEPOT_CENTER_MINERAL_X(-BLUE_DEPOT_CENTER_MINERAL_X.val),
        RED_DEPOT_CENTER_MINERAL_Y(-BLUE_DEPOT_CENTER_MINERAL_Y.val),
        RED_DEPOT_RIGHT_MINERAL_X(-BLUE_DEPOT_RIGHT_MINERAL_X.val),
        RED_DEPOT_RIGHT_MINERAL_Y(-BLUE_DEPOT_RIGHT_MINERAL_Y.val),

        RED_CRATER_LEFT_MINERAL_X(-BLUE_DEPOT_LEFT_MINERAL_Y.val),
        RED_CRATER_LEFT_MINERAL_Y(BLUE_DEPOT_LEFT_MINERAL_X.val),
        RED_CRATER_CENTER_MINERAL_X(-BLUE_DEPOT_CENTER_MINERAL_Y.val),
        RED_CRATER_CENTER_MINERAL_Y(BLUE_DEPOT_CENTER_MINERAL_X.val),
        RED_CRATER_RIGHT_MINERAL_X(-BLUE_DEPOT_RIGHT_MINERAL_Y.val),
        RED_CRATER_RIGHT_MINERAL_Y(BLUE_DEPOT_RIGHT_MINERAL_X.val);

        public final double val;
        MineralLocations (double i )  {val = i;}
    }
    enum Objectives { PARK, LAND, SAMPLE, DROP_TEAM_MARKER; }

    //openCV variables
    //We are using openCV to detect the location of the gold object.
    private int OPENCV_IMAGE_MIDDLE = 360; //the halfway point in the image, in y coords (since our phone is in landscape mode).
    OpenCV openCV; //the opencv image processing pipeline
    enum GoldLocation
    {
        LEFT,
        CENTER,
        RIGHT
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
        telemetry.addData("Init State", "Init Finished");
        telemetry.addData("Alliance", alliance.name());
        telemetry.addData("Side", startLocation.name());
        telemetry.addData("Delay Time", delayTime);
        telemetry.update();

        // Set last known encoder values
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        // Set IMU heading offset
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        servoJJ.setPosition(0.4);
        servoJJ2.setPosition(0.65);

        //initialize our openCV image processing pipeline
        //openCVInit();

        telemetry.clear();
        telemetry.update();
        telemetry.addLine("Initialized. Ready to start!");
    }

    public void openCVInit()
    {
        openCV = new OpenCV();
        openCV.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        openCV.enable();
        openCV.setShowCountours(true);
    }

    public void openCVDisable()
    {
        openCV.disable(); //no need to keep processing openCV frames.
    }

    public GoldLocation openCVLocateGold()
    {
        GoldLocation gold;

        Rect goldRect = openCV.getGoldRect();

        telemetry.addData("Gold",
                String.format(Locale.getDefault(), "(%d, %d)", (goldRect.x + goldRect.width / 2), (goldRect.y + goldRect.height / 2) ) );

        //This function assumes that the openCV frame contains the left and center objects only.
        //goldRect will be (0,0,0,0) if the yellow color is not found in the image; i.e., the gold must be on the right.

        //We are checking Y because our phone is in landscape mode.
        int midpoint = goldRect.y + goldRect.height / 2;

        if((midpoint > 0) && (midpoint < OPENCV_IMAGE_MIDDLE))
        {
            telemetry.addData("Position: ", "Left");
            gold = GoldLocation.LEFT;
        }
        else if(midpoint >= OPENCV_IMAGE_MIDDLE)
        {
            telemetry.addData("Position: ", "Center");
            gold = GoldLocation.CENTER;
        }
        else //the yellow color was not found in the image; gold must be on the right.
        {
            telemetry.addData("Position", "Right");
            gold = GoldLocation.RIGHT;
        }

        telemetry.update();

        return gold;
    }

    public void deployArm()
    {
        /*
        motorFlip.setTargetPosition(motorFlip.getCurrentPosition() - 600);
        while (Math.abs(motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) > 25)
            motorFlip.setPower(Math.min((motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) * (1.0 / 150), 0.2));
        motorFlip.setPower(0.0);
        */
        motorFlip.setTargetPosition(motorFlip.getCurrentPosition() - 600);
        while (Math.abs(motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) > 25)
            motorFlip.setPower(0.4);

        motorFlip.setPower(0.0);
        sleep(250);

        motorFlip.setTargetPosition(motorFlip.getCurrentPosition() - 300);
        while (Math.abs(motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) > 25)
            motorFlip.setPower(Math.min((motorFlip.getTargetPosition() - motorFlip.getCurrentPosition()) * (1.0 / 150), 0.05));
        motorFlip.setPower(0.0);

        motorFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void deployArmFAST()
    {
        motorFlip.setTargetPosition(motorFlip.getCurrentPosition() - 500);
        while (Math.abs(motorFlip.getCurrentPosition() - motorFlip.getTargetPosition()) > TOL)
        {
            motorFlip.setPower(0.5);
        }
        motorFlip.setPower(0.0);

        motorFlip.setTargetPosition(motorFlip.getCurrentPosition() + 690);
        while (Math.abs(motorFlip.getCurrentPosition() - motorFlip.getTargetPosition()) > TOL)
        {
            motorFlip.setPower(0.3);
        }
        motorFlip.setPower(0.0);
    }

    public void dropJJ()
    {
        //1 is completely up
        //-1 is completely down
        servoJJ.setPosition(0.50);
        sleep(600);
        servoJJ.setPosition(0.9);
        sleep(100);
        servoJJ.setPosition(0.95);
    }

    public void moveLift (int ticks)
    {
        while (Math.abs(ticks - motorLift.getCurrentPosition()) > TOL)
        {
            motorLift.setTargetPosition(motorLift.getCurrentPosition() + ticks);
            motorLift.setPower((motorLift.getTargetPosition() - motorLift.getCurrentPosition()) * (1 / 1000.0));
            sendTelemetry();
            idle();
        }
        stopLift();
    }
    public void stopLift()
    {
        motorLift.setPower(0.0);
        idle();
    }

    public void configureAutonomous()
    {
        // waste the zero index because we can't have zero delays
        delays.add(0);
        while(!doneSettingUp)
        {
            if(gamepad1.x)
                alliance = Alliance.BLUE;
                //means we are blue alliance
            else if (gamepad1.b)
                alliance = Alliance.RED;
                // means we are red alliance

            //means we are crater side
            if(gamepad1.dpad_left)
            {
                startLocation = StartLocations.CRATER;
                assist = Assist.NOT_ASSISTING;
            }
            //means we are depot side
            else if (gamepad1.dpad_right)
            {
                startLocation = StartLocations.DEPOT;
            }

            if (startLocation == StartLocations.DEPOT && gamepad1.a)
            {
                assist = Assist.ASSISTING;
            }
            else if (gamepad1.y)
            {
                assist = Assist.NOT_ASSISTING;
            }

            if(gamepad1.dpad_up)
            {
                numDelays++;
                delays.add(1);
                boolean customizingTime = true;

                while (!buttonsAreReleased(gamepad1))
                {
                    telemetry.update();
                    idle();
                }

                while(customizingTime)
                {
                    if(gamepad1.dpad_up)
                        delays.set(numDelays, delays.get(numDelays) + 1);
                    else if(gamepad1.dpad_down && delays.get(numDelays) >= 0)
                        delays.set(numDelays, delays.get(numDelays) - 1);
                    if(delays.get(numDelays) <= 0)
                    {
                        delays.remove(numDelays);
                        numDelays--;
                        customizingTime = false;
                    }
                    if(gamepad1.a)
                        customizingTime = false;
                    else if (gamepad1.start)
                    {
                        customizingTime = false;
                        doneSettingUp = true;
                    }

                    while (!buttonsAreReleased(gamepad1))
                    {
                        telemetry.update();
                        idle();
                    }

                    telemetry.addData("Delay Number", numDelays);
                    telemetry.addLine("Delay Increase/Decrease: Dpad Up / Down ");
                    telemetry.addLine("Press 'A' to confirm");
                    telemetry.addData("delay time", delays.get(numDelays));
                    telemetry.update();
                }
            }

            if(gamepad1.start)
                doneSettingUp = true;

            while (!buttonsAreReleased(gamepad1))
            {
                telemetry.update();
                idle();
            }

            // input information
            telemetry.addLine("Alliance Blue/Red: X/B");
            telemetry.addLine("Starting Position Crater/Depot: D-Pad Left/Right");
            telemetry.addLine("Add a delay: D-Pad Up");
            telemetry.addLine("After routine is complete and robot is on field, press Start");
            telemetry.addLine();

            // setup data
            telemetry.addData("Alliance", alliance.name());
            telemetry.addData("Side", startLocation.name());
            for (int i = 1; i <= delays.size() - 1 && delays.size() != 1; i++)
            {
                telemetry.addData("delay " + i, delays.get(i));
            }
            telemetry.update();

            idle();
        }

        // We could clear the telemetry at this point, but the drivers may want to see it
        telemetry.clear();

        telemetry.addLine("Setup complete. Initializing...");

        // Set coordinates based on alliance and starting location
        if(startLocation == StartLocations.DEPOT)
        {
            if(alliance == Alliance.RED)
            {
                robotX = StartLocations.RED_DEPOT_START_X.val;
                robotY = StartLocations.RED_DEPOT_START_Y.val;
                headingOffset = StartLocations.RED_DEPOT_START_ANGLE.val;
            }
            else if(alliance == Alliance.BLUE)
            {
                robotX = StartLocations.BLUE_DEPOT_START_X.val;
                robotY = StartLocations.BLUE_DEPOT_START_Y.val;
                headingOffset = StartLocations.BLUE_DEPOT_START_ANGLE.val;
            }
        }
        else if(startLocation == StartLocations.CRATER)
        {
            if (alliance == Alliance.RED)
            {
                robotX = StartLocations.RED_CRATER_START_X.val;
                robotY = StartLocations.RED_CRATER_START_Y.val;
                headingOffset = StartLocations.RED_CRATER_START_ANGLE.val;
            } else if (alliance == Alliance.BLUE)
            {
                robotX = StartLocations.BLUE_CRATER_START_X.val;
                robotY = StartLocations.BLUE_CRATER_START_Y.val;
                headingOffset = StartLocations.BLUE_CRATER_START_ANGLE.val;
            }
        }
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


    public void moveAuto(double x, double y, double speed, double minSpeed, double timeout) throws InterruptedException
    {
        newTargetFL = motorFL.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) + (int) Math.round(COUNTS_PER_MM * x * 1.15);
        newTargetFR = motorFR.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) - (int) Math.round(COUNTS_PER_MM * x * 1.15);
        newTargetBL = motorBL.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) - (int) Math.round(COUNTS_PER_MM * x * 1.15);
        newTargetBR = motorBR.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) + (int) Math.round(COUNTS_PER_MM * x * 1.15);
        runtime.reset();
        do
        {
            errorFL = newTargetFL - motorFL.getCurrentPosition();
            speedFL = Math.abs(errorFL * Kmove);
            speedFL = Range.clip(speedFL, minSpeed, speed);
            speedFL = speedFL * Math.signum(errorFL);

            errorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Math.abs(errorFR * Kmove);
            speedFR = Range.clip(speedFR, minSpeed, speed);
            speedFR = speedFR * Math.signum(errorFR);

            errorBL = newTargetBL - motorBL.getCurrentPosition();
            speedBL = Math.abs(errorBL * Kmove);
            speedBL = Range.clip(speedBL, minSpeed, speed);
            speedBL = speedBL * Math.signum(errorBL);

            errorBR = newTargetBR - motorBR.getCurrentPosition();
            speedBR = Math.abs(errorBR * Kmove);
            speedBR = Range.clip(speedBR, minSpeed, speed);
            speedBR = speedBR * Math.signum(errorBR);

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);
            idle();
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (Math.abs(errorFL) > TOL || Math.abs(errorFR) > TOL || Math.abs(errorBL) > TOL || Math.abs(errorBR) > TOL));

        stopDriving();
    }
    // normalizing the angle to be between -180 to 180
    public double adjustAngles(double angle)
    {
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
    }
    void imuPivot(double referenceAngle, double targetAngle, double MaxSpeed, double kAngle, double timeout)
    {
        runtime.reset();
        //counter-clockwise is positive
        double pivot;
        double currentRobotAngle;
        double angleError;

        targetAngle =  referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            targetAngle = adjustAngles(targetAngle);//Makes it so the target angle does not wrap
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            if (pivot >= 0.0)
            {
                pivot = Range.clip(pivot, 0.15, MaxSpeed);
            }
            else
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

        targetAngle =  referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            targetAngle = adjustAngles(targetAngle);//Makes it so the target angle does not wrap
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            if (pivot >= 0.0)
            {
                pivot = Range.clip(pivot, 0.15, MaxSpeed);
            }
            else
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

    // Makes robot drive to a point on the field
    void driveToPoint(double targetX, double targetY, double targetAngle) throws InterruptedException
    {
        updateRobotLocation();

        // Calculate how far we are from target point
        double distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double DISTANCE_TOLERANCE = 40; // In mm
        double ANGLE_TOLERANCE = 5; // In degrees

        // Run until robot is within tolerable distance and angle
        while(!(distanceToTarget < DISTANCE_TOLERANCE && deltaAngle < ANGLE_TOLERANCE) && opModeIsActive())
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

        while(Math.abs(deltaAngle) > ANGLE_TOLERANCE && opModeIsActive())
        {
            updateRobotLocation();

            // Recalculate how far away we are
            deltaAngle = subtractAngles(targetAngle, robotAngle);

            // Slow down as we approach target
            double turnPower = Range.clip(deltaAngle * TURN_POWER_CONSTANT, -1.0, 1.0);

            // Make sure turn power doesn't go below minimum power
            if(turnPower > 0 && turnPower < MIN_DRIVE_POWER)
                turnPower = MIN_DRIVE_POWER;
            else if(turnPower < 0 && turnPower > -MIN_DRIVE_POWER)
                turnPower = -MIN_DRIVE_POWER;

            // Set drive motor power
            driveMecanum(0.0, 0.0, turnPower);
            sendTelemetry();
            idle();
        }
        stopDriving();
    }

    // Updates robot's coordinates and angle
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
        deltaX *= MM_PER_TICK;
        deltaY *= MM_PER_TICK;

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

    public void stopDriving ()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }


///////////////////////////////////////////////////////////////////////////////////////////////////
//Autonomous phases
    public GoldLocation detectMineral() throws InterruptedException
    {
        //!!! should we be locating the gold AFTER we finish the move that we do below??
        GoldLocation position = openCVLocateGold();
        return position;
    }
    public GoldLocation landAndDetectMineral() throws InterruptedException
    {
        moveLift(6700);

        //!!! should we be locating the gold AFTER we finish the move that we do below??
        GoldLocation position = openCVLocateGold();

        sleep(100); //this can be removed?


        openCVDisable(); //stop openCV processing since we don't need it anymore.


        /*switch (alliance)
        {
            case RED:
                switch (startLocation)
                {
                    case DEPOT:
                        driveToPoint(robotX + 40, robotY - 40, headingOffset, 0.5);
                        break;
                    case CRATER:
                        driveToPoint(robotX - 40, robotY - 40, headingOffset, 0.5);
                        break;
                }
            case BLUE:
                switch (startLocation)
                {
                    case DEPOT:
                        driveToPoint(robotX - 40, robotY + 40, headingOffset, 0.5);
                        break;
                    case CRATER:
                        driveToPoint(robotX + 40, robotY + 40, headingOffset, 0.5);
                        break;
                }
        }*/

        return position;
    }


    public void knockOffLeftMineral() throws InterruptedException
    {
        switch (startLocation)
        {
            case DEPOT:
            {
                moveAuto(0, -280, 0.5, 0.3, 3.0);
                moveAuto(480, 0, 0.5, 0.3, 3.0);
                moveAuto(0, -660, 0.5, 0.3, 3.0);
                break;
                //driveToPoint(698.3, 1130.5, 45, 0.5);
            }
            case CRATER:
            {
                //driveToPoint(698.3, -1130.5, 315, 0.5);

            }
        }
    }

    public void knockOffCenterMineral() throws InterruptedException
    {
        switch (startLocation)
        {
            case DEPOT:
            {
                moveAuto(0, -1270, 0.5, 0.35, 3.0);
                moveAuto(0, 75, 0.7, 0.35, 3.0);
                break;
                //driveToPoint(958.8, 870, 45, 0.5);
            }
            case CRATER:
            {
                //driveToPoint(958.8, -870, 315, 0.5);
            }
        }
    }

    public void knockOffRightMineral() throws InterruptedException
    {
        switch (startLocation)
        {
            case DEPOT:
            {
                moveAuto(0, -280, 0.5, 0.3, 3.0);
                moveAuto(-480, 0, 0.5, 0.3, 3.0);
                moveAuto(0, -660, 0.5, 0.3, 3.0);
                //driveToPoint(-1219.2, -609.6, 215, 0.5);
            }
            case CRATER:
            {
                //driveToPoint(-1219.2, 609.6, 135, 0.5);
            }
        }
    }

    public void sendTelemetry()
    {
        // Inform drivers of robot location
        telemetry.addData("X", robotX);
        telemetry.addData("Y", robotY);
        telemetry.addData("Robot Angle", robotAngle);

        telemetry.addData("Lift Encoder", motorLift.getCurrentPosition());

        // Debug info
        /*telemetry.addData("FL Encoder", motorFL.getCurrentPosition());
        telemetry.addData("FR Encoder", motorFR.getCurrentPosition());
        telemetry.addData("BL Encoder", motorBL.getCurrentPosition());
        telemetry.addData("BR Encoder", motorBR.getCurrentPosition());*/

        telemetry.update();
    }
}
