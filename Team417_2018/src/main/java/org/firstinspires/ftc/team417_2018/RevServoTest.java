package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name="Rev Servo Test", group="Swerve")
//@Disabled
public class RevServoTest extends LinearOpMode
{
    //Servo rev1 = null;
    //Servo rev2 = null;

    CRServo hanger = null; // hub 2 port 5

    DcMotor core1 = null; // hub 1 port 3
    DcMotor core2 = null; // hub 2 port 3

    DcMotor arm1 = null; // hub 1 port 4
    DcMotor arm2 = null; // hub 2 port 4

    CRServo vex1 = null; // port 0
   // CRServo vex2 = null; //port 1

    double curExtendPos1 = 0.0;
    double curExtendPos2 = 0.0;

    double core1Power;
    double core2Power;
    double ADAGIO_POWER = 0.2;

    double rev1pos = 0.02;
    double rev2pos = 0.98;

    public void runOpMode()
    {
        core1 = hardwareMap.dcMotor.get("core1");
        core2 = hardwareMap.dcMotor.get("core2");

        vex1 = hardwareMap.crservo.get("vex1");

        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");

        hanger = hardwareMap.crservo.get("hanger");

        core1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        core2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        core1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        core2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("init", "done");

        telemetry.update();
        //rev1.setPosition(0.02);
        //rev2.setPosition(0.98);

        vex1.setPower(0.0);
        //vex2.setPower(0.0);

        waitForStart();

        while (opModeIsActive())
        {
            curExtendPos1 = core1.getCurrentPosition();
            curExtendPos1 = core2.getCurrentPosition();
            // control REV smart servos
            /*
            if(gamepad1.dpad_up)
            {
                rev1pos += 0.01;
            }
            else if (gamepad1.dpad_down)
            {
                rev1pos -= 0.01;
            }
            //rev1.setPosition(rev1pos);
            //rev2.setPosition(rev2pos);

            if(gamepad1.dpad_right)
            {
                rev2pos += 0.01;
            }
            else if (gamepad1.dpad_left)
            {
                rev2pos -= 0.01;
            }
            */
            // control core hex motors
            if (gamepad2.right_trigger > 0) // extend the collector
            {
                core1Power = -gamepad2.right_trigger;
                core2Power = -gamepad2.right_trigger;

            }
            else if (gamepad2.left_trigger > 0) // pull the collector in
            {
                core1Power = gamepad2.left_trigger;
                core2Power = gamepad2.left_trigger;
            }
            if (gamepad2.dpad_left)
            {
                core1.setPower(Range.clip(core1Power, -ADAGIO_POWER, ADAGIO_POWER));
                core2.setPower(Range.clip(core1Power, -ADAGIO_POWER, ADAGIO_POWER));
            }
            else
            {
                core1Power = 0.0;
                core2Power = 0.0;
            }
            core1.setPower(core1Power);
            core2.setPower(core2Power);

            // control AM 3.7 motors
            if (gamepad2.right_stick_y != 0)
            {
                arm1.setPower(gamepad2.right_stick_y);
                arm2.setPower(-gamepad2.right_stick_y);
            }
            else if (gamepad2.right_stick_y != 0 && gamepad2.dpad_left)
            {
                arm1.setPower(Range.clip(gamepad2.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER));
                arm2.setPower(Range.clip(-gamepad2.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER));
            }
            else
            {
                arm1.setPower(0.0);
                arm2.setPower(0.0);
            }
            telemetry.addData("core1",curExtendPos1);
            telemetry.addData("core2",curExtendPos2);

            if (gamepad2.left_bumper)
            {
                hanger.setPower(0.99);
            }
            else if (gamepad2.right_bumper)
            {
                hanger.setPower(-0.99);
            }
            else
            {
                hanger.setPower(0.0);
            }

            if(gamepad2.b)
        {
            vex1.setPower(0.79);
        }
        else if (gamepad2.x)
        {
            vex1.setPower(-0.79);
        }
        else
        {
            vex1.setPower(0);

        }

            //telemetry.addData("rev1", rev1.getPosition());
            //telemetry.addData("rev2", rev1.getPosition());
            telemetry.update();
        }
    }
}
