package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "New Robot TeleOp")
public class FlynnTestNewRobot extends MasterTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeNewHardware();

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collectorMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collectorMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotorL.setTargetPosition(0);
        liftMotorR.setTargetPosition(0);
        liftMotorL.setPower(0.5);
        liftMotorR.setPower(0.5);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Init:", "Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            driveRobot();
            newCollector(0.3);
            newStacker(0.3, 0.05);
            foundationPullers();
            updateTelemetry();
        }
    }
}
