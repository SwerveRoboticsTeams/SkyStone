package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Linear Slide Test")
@Disabled
public class LinearSlideTest extends MasterTeleOp
{
    int targetPos = 0;

    public void runOpMode() throws InterruptedException
    {
        super.initializeHardware();

        telemetry.addData("Init","done");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad2.right_trigger != 0)
            {
                core2.setPower(0.7);
            }
            else if (gamepad2.left_trigger != 0)
            {
                isExtending = false;
                core2.setPower(-0.8);
            }
            else if (gamepad2.left_trigger==0 && gamepad2.right_trigger==0 && !isExtending)
            {
                core2.setPower(0.0);
            }

            // Press button y to toggle up and down
            if (gamepad2.y && !isExtending)
            {
                isYButtonPressed = true;
                isExtending = !isExtending;
            }
            isYButtonPressed = gamepad2.y;
            if (isExtending) core2.setPower(0.025); // extends with toggle

            telemetry.addData("core2", core2.getCurrentPosition());
            telemetry.update();

        }
    }
}
