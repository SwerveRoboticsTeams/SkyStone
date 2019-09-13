package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Rect;

import java.util.Locale;

abstract public class MasterAutonomous extends MasterOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime autoRuntime = new ElapsedTime();

    String goldLocation = "UNKNOWN";
    String lastSeenGold = "UNKNOWN";

    private int OPENCV_IMAGE_MIDDLE = 360;

    //arm
    private int curLiftPos = 0;
    //slides
    private int curExtendPos = 0;
    // hanger
    private int curHangerPos = 0;


    boolean isLogging = true;
    boolean isPosCrater = true;

    boolean isLeftGold = false;
    boolean isCenterGold = true;
    boolean isRightGold = false;

    int threshold = 90;
    int delay = 0;

    // speed is proportional to error
    double Kmove = 1.0f/1200.0f;
    double Kpivot = 1.0f/150.0f;

    double TOL = 100.0; // this is in encoder counts, and for our robot with 4 inch wheels, it's 0.285 mm in one encoder count
    double TOL_ANGLE = 5;

    double MINSPEED = 0.25;
    double PIVOT_MINSPEED = 0.2;

    // for gold detection
    Dogeforia vuforia;
    WebcamName webcamName;
    OpenCVDetect OpenCV_detector;

    // VARIABLES FOR AUTONOMOUS GG
    int curGGPos;
    int errorMinGG;
    int errorMaxGG;
    int minGGPos = -160; // a bit less than the original starting position of zero (where we start it)
    int maxGGPos = -500; // maxGGPos equals the # rev to close/open GG (13 rev) times 44.4 counts per rev
    double speedGG;
    double KGlyph = 1/1000.0;
    int maxGMPos = 500;
    int curGMPos;

    // VARIABLES FOR MOVE/ALIGN METHODS
    int pivotDst;

    int newTargetFL;
    int newTargetBL;
    int newTargetFR;
    int newTargetBR;

    int errorFL;
    int errorFR;
    int errorBL;
    int errorBR;

    double speedFL;
    double speedFR;
    double speedBL;
    double speedBR;

    double speedAbsFL;
    double speedAbsFR;
    double speedAbsBL;
    double speedAbsBR;

    double curTurnAngle;
    double pivotSpeed;
    double errorAngle;

    double avgDistError;

    public void autoInitializeRobot()
    {
        super.initializeHardware();
        rev1.setPosition(INIT_REV_POS);
        marker.setPosition(MARKER_LOW);

        // zero the motor controllers before running; we don't know if motors start out at zero
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void InitializeDetection()
    {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key - Truong
        parameters.vuforiaLicenseKey = "AdTQ1zz/////AAABmcsU0JSsfUAAnZVqALQznloFPRzK4IDs9AKHiU80F9ncKlBHZBPTN3XWUSLbcsKUfy/iW4P/y64OCRHrAiUTE430LhFnx8rGRtKUv8P03XTaE11Xj9gbN5vThAIBcrnk/CovUIBFJjptCseciz/akh2mWHAlNznx5kWdP0QbFRi9i6fZffoHXaBXYERvzyK/wYYxMLuwVL+qBGIuRzJRS4f2b6RZ8cq/SEs6Ulfg5HQgV24KqFA65+T7iGXKCrdQMi0eUN0Oc4DmrKrHKF55bEtA108/jh8cz1tAwsrAvjle6JUX+yUQ4RDX8Zv/GpuWdek3VFGxumvh8EdQAmZqUmaWdcrpHLXMcftKdOjyvyUf";
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Initialize the OpenCV_detector
        OpenCV_detector = new OpenCVDetect();

        // fullscreen display:
        //   app crashes if screen orientation switches from portrait to landscape
        //   screen goes to sleep, and webcam turns off a few minutes after init, and after play
        //OpenCV_detector.init(hardwareMap.appContext, ActivityViewDisplay.getInstance(), 0, true);

        OpenCV_detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        OpenCV_detector.setThreshold(threshold);
        OpenCV_detector.setShowCountours(false);
        // Set the OpenCV_detector
        vuforia.setDogeCVDetector(OpenCV_detector);
        vuforia.enableDogeCV();
        // don't show Vuforia vuforia.showDebug();
        vuforia.start();

        telemetry.addData("Done: ", "initializing");
        telemetry.update();

        // fullscreen display:
        //   app crashes if screen orientation switches from portrait to landscape

        //   screen goes to sleep, and webcam turns off a few minutes after init, and after play
        //OpenCV_detector.init(hardwareMap.appContext, ActivityViewDisplay.getInstance(), 0, true);
    }


    // won't display Gold position (left, center, right) if gold mineral doesn't meet the vertical cutting at 400 pixels or more
    public void locateGold()
    {
        // right gold was located at (700,380)
        if (((OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2) >= 600) &&  (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2) >= 375 )
        {
            isLeftGold = false;
            isCenterGold = false;
            isRightGold = true;
            lastSeenGold = "RIGHT";
            goldLocation = "RIGHT";
        }
        // left gold was located at (110,380)
        else if (((OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2) <= 200)  &&  (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2) >= 375 )
        {
            isLeftGold = true;
            isCenterGold = false;
            isRightGold = false;
            lastSeenGold = "LEFT";
            goldLocation = "LEFT";
        }
        // center gold was located at (420,380)
        else if (((OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2) >= 350) && ((OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2) <= 500)  &&  (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2) >= 375 )
        {
            isLeftGold = false;
            isCenterGold = true;
            isRightGold = false;
            lastSeenGold = "CENTER";
            goldLocation = "CENTER";
        }
        else
        {
            lastSeenGold = "UNKNOWN";
        }

        //if (lastSeenGold == "UNKNOWN") lastSeenGold = goldLocation;

        telemetry.addData("Webcam sees", lastSeenGold);
        telemetry.addData("Going for", goldLocation);
    }

    // drive forwards/backwards/horizontal left and right function
    public void move(double x, double y, double minSpeed, double maxSpeed, double timeout) throws InterruptedException
    {
        newTargetFL = motorFL.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y * SCALE_OMNI) + (int) Math.round(COUNTS_PER_MM * x * SCALE_OMNI);
        newTargetFR = motorFR.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y * SCALE_OMNI) - (int) Math.round(COUNTS_PER_MM * x * SCALE_OMNI);
        newTargetBL = motorBL.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y * SCALE_OMNI) - (int) Math.round(COUNTS_PER_MM * x * SCALE_OMNI);
        newTargetBR = motorBR.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y * SCALE_OMNI) + (int) Math.round(COUNTS_PER_MM * x * SCALE_OMNI);

        runtime.reset(); // used for timeout

        // wait until the motors reach the position
        do
        {
            errorFL = newTargetFL - motorFL.getCurrentPosition();
            speedFL = Math.abs(errorFL * Kmove);
            speedFL = Range.clip(speedFL, minSpeed, maxSpeed);
            speedFL = speedFL * Math.signum(errorFL);

            errorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Math.abs(errorFR * Kmove);
            speedFR = Range.clip(speedFR, minSpeed, maxSpeed);
            speedFR = speedFR * Math.signum(errorFR);

            errorBL = newTargetBL - motorBL.getCurrentPosition();
            speedBL = Math.abs(errorBL * Kmove);
            speedBL = Range.clip(speedBL, minSpeed, maxSpeed);
            speedBL = speedBL * Math.signum(errorBL);

            errorBR = newTargetBR - motorBR.getCurrentPosition();
            speedBR = Math.abs(errorBR * Kmove);
            speedBR = Range.clip(speedBR, minSpeed, maxSpeed);
            speedBR = speedBR * Math.signum(errorBR);

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);

            telemetry.log().add(String.format("errorFL: %d, speedFL: %f" , errorFL, speedFL));
            idle();
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (Math.abs(errorFL) > TOL && Math.abs(errorFR) > TOL && Math.abs(errorBL) > TOL && Math.abs(errorBR) > TOL));
        // all of the motors must reach their tolerance before the robot exits the loop

        // stop the motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }


    // a combination of both the align and pivot function
    // angle has to be small otherwise won't work, this function translates and while maintaining a certain angle
    public void moveMaintainHeading(double x, double y, double pivotAngle, double refAngle, double minSpeed, double maxSpeed, double timeout)
    {
        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        curTurnAngle = imu.getAngularOrientation().firstAngle - refAngle;
        curTurnAngle = adjustAngles(curTurnAngle);
        errorAngle = pivotAngle - curTurnAngle;

        pivotDst = (int) ((errorAngle / 360.0) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

        int xTarget = (int) Math.round(COUNTS_PER_MM * (x * SCALE_OMNI));
        int yTarget = (int) Math.round(COUNTS_PER_MM * (y * SCALE_OMNI));
        newTargetFL = motorFL.getCurrentPosition() + xTarget + yTarget + pivotDst;
        newTargetFR = motorFR.getCurrentPosition() - xTarget + yTarget - pivotDst;
        newTargetBL = motorBL.getCurrentPosition() - xTarget + yTarget + pivotDst;
        newTargetBR = motorBR.getCurrentPosition() + xTarget + yTarget - pivotDst;

        runtime.reset(); // reset timer, which is used for loop timeout below

        // wait until the motors reach the position
        // adjust robot angle during movement by adjusting speed of motors
        do
        {
            // read the real current angle and compute error compared to ref angle
            curTurnAngle = imu.getAngularOrientation().firstAngle - refAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            errorAngle =  pivotAngle - curTurnAngle;
            pivotSpeed = errorAngle * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, -0.4, 0.4); // limit max pivot speed
            // pivotSpeed is added to each motor's movement speed

            errorFL = newTargetFL - motorFL.getCurrentPosition();
            speedFL = Kmove * errorFL;  // movement speed proportional to error
            speedAbsFL = Math.abs(speedFL);
            // clip abs(speed) MAX speed minus 0.3 to leave room for pivot factor
            speedAbsFL = Range.clip(speedAbsFL, minSpeed, maxSpeed);
            speedFL = speedAbsFL * Math.signum(speedFL);  // set sign of speed
            speedFL += pivotSpeed;  // combine movement and pivot speeds

            errorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Kmove * errorFR;
            speedAbsFR = Math.abs(speedFR);
            speedAbsFR = Range.clip(speedAbsFR, minSpeed, maxSpeed);  // clip abs(speed)
            speedFR = speedAbsFR * Math.signum(speedFR);
            speedFR -= pivotSpeed;  // combine movement and pivot speeds

            errorBL = newTargetBL - motorBL.getCurrentPosition();
            speedBL = Kmove * errorBL;
            speedAbsBL = Math.abs(speedBL);
            speedAbsBL = Range.clip(speedAbsBL, minSpeed, maxSpeed);  // clip abs(speed)
            speedBL = speedAbsBL * Math.signum(speedBL);
            speedBL += pivotSpeed;  // combine movement and pivot speeds

            errorBR = newTargetBR - motorBR.getCurrentPosition();
            speedBR = Kmove * errorBR;
            speedAbsBR = Math.abs(speedBR);
            speedAbsBR = Range.clip(speedAbsBR, minSpeed, maxSpeed);
            speedBR = speedAbsBR * Math.signum(speedBR);
            speedBR -= pivotSpeed; // combine movement and pivot speeds

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(Range.clip(speedBL,-0.6,0.6));
            motorBR.setPower(speedBR);

            avgDistError = (Math.abs(errorFL) + Math.abs(errorFR) + Math.abs(errorBL) + Math.abs(errorBR)) / 4.0;

            if (Math.abs(avgDistError) < 120.0)
            {
                sleep(50);
                // stop motors
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                sleep(50);
            }

            idle();
        }
        while ( (opModeIsActive()) &&
                (runtime.seconds() < timeout) &&
                (
                        // exit the loop when one of the motors achieve their tolerance
                        //( (Math.abs(errorFL) > TOL) && (Math.abs(errorFR) > TOL) && (Math.abs(errorBL) > TOL) && (Math.abs(errorBR) > TOL) )
                        avgDistError > TOL
                                || (Math.abs(errorAngle) > TOL_ANGLE)
                )
                );

        // stop the motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }



    // this method drives for seconds, and it can only pivot
    public void moveTimed(double yPower, int milliSeconds) throws InterruptedException
    {
        powerFL = yPower;
        powerFR = yPower;
        powerBL = yPower;
        powerBR = yPower;

        // turn on power
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(Range.clip(powerBL,-0.6,0.6));
        motorBR.setPower(powerBR);

        // let it run for x seconds
        sleep(milliSeconds);
        // stop the motors after x seconds
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    // pivot using IMU, but with a reference start angle, but this angle has to be determined (read) before this method is called
    public void pivotWithReference(double targetAngle, double refAngle, double minSpeed, double maxSpeed)
    {
        double pivotSpeed;
        double currentAngle;
        double errorAngle;

        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // read angle, record in starting angle variable
        // run motor
        // loop, current angle - start angle = error
        // if error is close to 0, stop motors

        do
        {
            currentAngle = adjustAngles(imu.getAngularOrientation().firstAngle - refAngle);
            errorAngle = adjustAngles(currentAngle - targetAngle);
            pivotSpeed = Math.abs(errorAngle) * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, minSpeed, maxSpeed); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(errorAngle); // set the sign of speed

            // positive angle means CCW rotation
            motorFL.setPower(pivotSpeed);
            motorFR.setPower(-pivotSpeed);
            motorBL.setPower(Range.clip(pivotSpeed,-0.6,0.6));
            motorBR.setPower(-pivotSpeed);

            // allow some time for IMU to catch up
            if (Math.abs(errorAngle) < 5.0)
            {
                sleep(15);
                // stop motors
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                sleep(150);
            }
/*
            sleep(100);
            motorFL.setPower(0.0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
*/
            if (isLogging) telemetry.log().add(String.format("StartAngle: %f, CurAngle: %f, error: %f", refAngle, currentAngle, errorAngle));
            idle();

        } while (opModeIsActive() && (Math.abs(errorAngle) > TOL_ANGLE));

        // stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    enum GoldLocation
    {
        LEFT,
        CENTER,
        RIGHT
    }
    public void land()
    {
        do
        {
            curLiftPos = arm1.getCurrentPosition();
            arm1.setPower(-0.2);
            arm2.setPower(0.2);
        }
        while (curLiftPos > -1030);
        arm1.setPower(0.0);
        arm2.setPower(0.0);

        sleep(50);
    }
    public void drop()
    {
        rev1.setPosition(0.0);
        lowerArm();
        vex1.setPower(0.79);
        sleep(2000);
        vex1.setPower(0.0);
    }
    public void landEncoder(int initPos, int finalPos)
    {
        int hangerTargetPos = hanger.getCurrentPosition() + initPos;
        do
        {
            curHangerPos = hanger.getCurrentPosition();
            hanger.setPower(0.99);
        }
        while (curHangerPos < hangerTargetPos);
        hanger.setPower(0.0);

        do
        {
            curLiftPos = arm1.getCurrentPosition();
            arm1.setPower(-0.55);
            arm2.setPower(0.55);
        }
        while (curLiftPos > -1030);
        arm1.setPower(0.0);
        arm2.setPower(0.0);

        sleep(50);

        hangerTargetPos = hanger.getCurrentPosition() + finalPos + initPos;
        do
        {
            curHangerPos = hanger.getCurrentPosition();
            hanger.setPower(0.99);
        }
        while (curHangerPos < hangerTargetPos);
        hanger.setPower(0.0);
    }
    public void lowerSmallAmt() {
        do
        {
            curLiftPos = arm1.getCurrentPosition();
            arm1.setPower(0.35);
            arm2.setPower(-0.35);
        }
        while (curLiftPos < -150);
        arm1.setPower(0.0);
        arm2.setPower(0.0);
    }

    public void end()
    {
        extend();
        lowerSmallAmt();
        vex1.setPower(-0.79);
        if (autoRuntime.milliseconds() < 29999) vex1.setPower(0.0);
    }

    public void lowerArm()
    {
        do
        {
            curLiftPos = arm1.getCurrentPosition();
            arm1.setPower(0.35);
            arm2.setPower(-0.35);
        }
        while (curLiftPos < -210);
        arm1.setPower(0.0);
        arm2.setPower(0.0);
    }
    public void extend()
    {
        do
        {
            curExtendPos = core2.getCurrentPosition();
            core2.setPower(0.99);
        }
        while (curExtendPos < 600 );
        core2.setPower(0.0);
    }
    public void reset() // we run this after Auto to reset the hanger and the arm motors
    {
        hanger.setPower(-0.96);
        sleep(3300);
        hanger.setPower(0.0);


    }

}
