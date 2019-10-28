package org.firstinspires.ftc.team8923_2019 ;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp Competition")

public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode()
    {


        initHardware();
        waitForStart();

        while (opModeIsActive())
        {
            Variables.ARM_MOTOR_TICKS = motorArm.getCurrentPosition() - Constants.ARM_STARTING_TICKS;

            telemetry.addData("armStartingTicks", Constants.ARM_STARTING_TICKS);
            telemetry.addData("armTicks", Variables.ARM_MOTOR_TICKS);

            telemetry.update();
            driveMecanumTeleOp();
            runIntake();
            runClaw();
           // sendTelemetry();

            idle();
        }
    }
    private double map(double value, double minValue, double maxValue, double minMappedValue, double maxMappedValue)
    {
        double valueDifference = maxValue - minValue;
        double percentValueDifference = (value - minValue) / valueDifference;
        double mappedDifference = maxMappedValue - minMappedValue;

        return percentValueDifference * mappedDifference;
    }
}
