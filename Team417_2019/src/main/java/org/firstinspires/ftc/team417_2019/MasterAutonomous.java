package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.team417_2019.ImageRecognition.Dogeforia;
import org.firstinspires.ftc.team417_2019.ImageRecognition.OpenCVDetect;
import java.util.List;


abstract public class MasterAutonomous extends MasterOpMode
{
    // timers
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime autoRuntime = new ElapsedTime();

    // speed constants to make sure speed is proportional to error (slow down for precision)
    double Kmove = 1.0f/1100.0f;
    double Kpivot = 1.0f/100.0f;
    // this is in encoder counts, and for our robot with 4 inch wheels, it's 0.285 mm in one encoder count
    double distanceTolerance = 0.5;
    double angleTolerance = 3;

    boolean isLogging = true;

    // Detection
    OpenCVDetect findSkystone;
    Dogeforia vuforia;
    WebcamName webcamName;


    public static final String VUFORIA_KEY =
            "AQdgAgj/////AAABmZGzg951/0AVjcK/+QiLWG1Z1PfbTwUouhED8hlwM6qrpAncj4xoMYYOUDxF+kreiazigY0q7OMa9XeMyxNlEQvyMFdefVUGSReIxJIXYhFaru/0IzldUlb90OUO3+J4mGvnzrqYMWG1guy00D8EbCTzzl5LAAml+XJQVLbMGrym2ievOij74wabsouyLb2HOab5nxk0FycYqTWGhKmS7/h4Ddd0UtckgnHDjNrMN4jqk0Q9HeTa8rvN3aQpSUToubAmfXe6Jgzdh2zNcxbaNIfVUe/6LXEe23BC5mYkLAFz0WcGZUPs+7oVRQb7ej7jTAJGA6Nvb9QKEa9MOdn0e8edlQfSBRASxfzBU2FIGH8a";

    public void autoInitializeRobot()
    {
        super.initializeHardware();

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
    }

    // todo Add turnSatisfactionCounter.
    // todo DON'T USE -180 WITH THIS METHOD, causes a wrap-around issue in the adjustAngles method
    // pivot using IMU, but with a reference start angle, but this angle has to be determined (read) before this method is called
    public void pivot(double targetAngle, double maxSpeed)
    {
        double pivotSpeed;
        double currentAngle;
        double errorAngle;
        int satisfactionCounter = 0;

        // read angle, record in starting angle variable
        // run motor
        // loop, current angle - start angle = error
        // if error is close to 0, stop motors

        do
        {
            robot.updatePosition();

            errorAngle = adjustAngles(robot.curAngle - targetAngle);
            turnFilter.roll(errorAngle);
            pivotSpeed = turnFilter.getFilteredValue();
            pivotSpeed = Range.clip(pivotSpeed, -maxSpeed, maxSpeed); // limit abs speed

            // add to angle satisfactionCounter if error is in certain range
            if (Math.abs(errorAngle) < 1) {
                satisfactionCounter++;
            }

            accelerationFilter.roll(pivotSpeed);
            if (Math.abs(robot.curAngle) < Math.abs(targetAngle) / 2) {
                pivotSpeed = accelerationFilter.getFilteredValue();
            }
            mecanumDrive(0, 0, pivotSpeed);

            telemetry.addData("CurAngle", robot.curAngle);
            telemetry.addData("ErrorAngle", errorAngle);
            telemetry.update();

            idle();

        } while (opModeIsActive() && (Math.abs(errorAngle) > angleTolerance)/* || (satisfactionCounter < 10)*/);

        // stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    /*
       x and y are robot's current location
       targetx and target y is the location you want to go to
       current angle is usually the reference angle/ angle you are currently facing
       add a speed such that the robot does not overpower and stays in -1.0 and 1.0 range
    */
    // todo work on implementing movement using "driveMecanum" MasterOpMode
    public void move(double targetX, double targetY, double maxSpeed) throws InterruptedException {

        double movingPower;
        double turningPower;
        double errorX;
        double errorY;
        double distanceToTarget;
        double angleToTarget;
        double errorAngle;
        // getting initial distance
        double initialAngle = robot.curAngle;
        double initialDistanceToTarget = Math.hypot(targetX - robot.currentX, targetY - robot.currentY);

        //This clears any residual vales in our PID loop
        turnFilter.reset();
        moveFilter.reset();

        do {

            robot.updatePosition();

            // cos = adjacent/hypotenuse
            // math .cos returns in radians so convert it back to degrees
            // double errorX = Math.cos(errorAngle * Math.PI/180)  * distanceToTarget;
            //errorX = targetX - ( 0.5 * robot.currentX + 0.5 * robot.currentX);
            errorX = targetX - robot.currentX;

            // sin = opposite/ hypotenuse
            //double errorY = Math.sin(errorAngle * Math.PI/180) * distanceToTarget;
            //errorY = targetY - ( 0.5 * robot.currentX - 0.5 * robot.currentY);
            errorY = targetY - robot.currentY;


            // find distance to target with shortcut distance formula
            distanceToTarget = Math.hypot(errorX, errorY);
            // find angle using arc tangent 2 to preserve the sign and find angle to the target
            angleToTarget = Math.atan2(errorY, errorX);
            // adjust angle that you need to turn so it is not greater than 180
            errorAngle = adjustAngles(robot.curAngle - initialAngle);

            // scale vector
            // make sure the power is between 0-1 but maintaining x and y power ratios with the total magnitude
            moveFilter.roll(distanceToTarget);
            turnFilter.roll(errorAngle);

            // Get filtered movement and turning powers.  Also clip movement power to ensure we aren't moving too fast.
            movingPower = Range.clip(moveFilter.getFilteredValue(), -maxSpeed, maxSpeed);
            turningPower = turnFilter.getFilteredValue();

            // roll movingPower
            accelerationFilter.roll(movingPower);
            // limiting movement power acceleration if far away enough from target
            if (distanceToTarget > initialDistanceToTarget / 2 ) {
                movingPower = accelerationFilter.getFilteredValue();
            }
            mecanumDrive(angleToTarget, movingPower, turningPower);

            telemetry.addData("motorFL", motorFL.getCurrentPosition());
            telemetry.addData("motorBL", motorBL.getCurrentPosition());
            telemetry.addData("motorFR", motorFR.getCurrentPosition());
            telemetry.addData("motorBR", motorBR.getCurrentPosition());
            telemetry.addData("errorX", errorX);
            telemetry.addData("errorY", errorY);
            telemetry.addData("errorAngle", errorAngle);
            telemetry.update();

            idle();
        } while ((Math.abs(errorAngle) > angleTolerance || distanceToTarget > distanceTolerance) && opModeIsActive());
    }


    // reset robot after auto
    public void reset() {}

}
