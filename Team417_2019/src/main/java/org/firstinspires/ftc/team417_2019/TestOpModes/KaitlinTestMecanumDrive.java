package org.firstinspires.ftc.team417_2019.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417_2019.MasterTeleOp;

@Disabled
@TeleOp (name = "Test Mecanum Drive")
abstract public class KaitlinTestMecanumDrive extends MasterTeleOp
{

    void mecanumDrive () {

        double x;
        double y;
        double pivotPower;

        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        pivotPower = gamepad1.right_stick_x;

        double powerFL = y + x + pivotPower;
        double powerBL = y - x + pivotPower;
        double powerFR = y - x - pivotPower;
        double powerBR = y + x - pivotPower;

    }

}
