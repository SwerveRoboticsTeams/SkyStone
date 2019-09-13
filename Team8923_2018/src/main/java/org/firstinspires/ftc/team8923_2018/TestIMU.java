package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IMU Test", group = "Test")
@Disabled
public class TestIMU extends MasterTeleOp
{
    BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    @Override
    public void runOpMode()
    {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("Imu Angle: ", imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }

    }
}
