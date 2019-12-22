package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

abstract public class LiftMotorTestMode extends LinearOpMode
{
    DcMotor liftMotor0, liftMotor1;

    DriverInput driver;
    List<ConcurrentOperation> callback;

    ElapsedTime timer = new ElapsedTime();

    public void initialize()
    {
        driver = new DriverInput(gamepad1);

        callback = new ArrayList<>();

        liftMotor0 = hardwareMap.dcMotor.get("liftMotor0");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");

        liftMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        callback.add(driver);
    }

    public void updateCallback(double eTime)
    {
        for (ConcurrentOperation item : callback)
        {
            item.update(eTime);
        }
    }
}
