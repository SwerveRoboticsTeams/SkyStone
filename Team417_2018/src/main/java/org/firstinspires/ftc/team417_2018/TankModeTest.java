package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


//@TeleOp(name="TankModeTest")
//@Disabled
public class TankModeTest extends LinearOpMode {


    DcMotor motorFL =null;
    DcMotor motorFR =null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;

    double leftPower;
    double rightPower;

    @Override
    public void runOpMode() {

        motorFL  = hardwareMap.dcMotor.get("motorFL");
        motorFR  = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");


        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry


            leftPower = -gamepad1.left_stick_y;
            rightPower =  -gamepad1.right_stick_y;

           // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            motorFL.setPower(Range.clip(leftPower,-1.0,1.0));
            motorFL.setPower(Range.clip(leftPower,-1.0,1.0));
            motorBL.setPower(Range.clip(rightPower,-1.0,1.0));
            motorBR.setPower(Range.clip(rightPower,-1.0,1.0));
           // motorBL.setPower(leftPower);
           // motorFR.setPower(rightPower);
           // motorBR.setPower(rightPower);

            idle();
        }
    }
}
