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

       // double refereneAngle = imu.getAngularOrientation().secondAngle;

        while (opModeIsActive())
        {
            Variables.ARM_MOTOR_TICKS = motorArm.getCurrentPosition() - Constants.ARM_STARTING_TICKS;


            driveMecanumTeleOp();
            //runIntake();
            runClaw();
            adjustClawPosition();
            toggleGrabber();
            sendTelemetry();
            toggleFoundationServos();
            resetArmTicksToZero();
            telemetry.clear();
            //telemetry.addData("reference angle: ", imu.getAngularOrientation());
            telemetry.update();
            idle();
        }
    }

}
