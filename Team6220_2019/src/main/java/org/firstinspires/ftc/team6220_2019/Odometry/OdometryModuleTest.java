package org.firstinspires.ftc.team6220_2019.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class is meant for testing the functionality of the encoder on an odometry module attached to a single REV hub.
 * It is not meant to be used to test odometry navigation.
 */
@TeleOp(name = "Odometry Module Test")
public class OdometryModuleTest extends LinearOpMode
{
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor liftMotor1;
    DcMotor liftMotor2;
    DcMotor collectorLeft;
    DcMotor collectorRight;

    public void runOpMode() throws InterruptedException
    {
        // Initialize encoders
        // Motor initializations--------------------------------------------
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");

        collectorLeft = hardwareMap.dcMotor.get("collectorLeft");
        collectorRight = hardwareMap.dcMotor.get("collectorRight");


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // RM—Connects to encoder for right module
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // RM
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collectorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // SM—Connects to encoder for strafe module
        collectorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // SM
        collectorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 1st lift motor runs using encoder, but 2nd one does not.
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // LM—Connects to encoder for left module

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // LM

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //-----------------------------------------------------------------

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Strafe Module Position: ", collectorLeft.getCurrentPosition());
            telemetry.addData("Left Module Position: ", liftMotor2.getCurrentPosition());
            telemetry.addData("Right Module Position: ", motorFL.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
