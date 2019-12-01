package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Test Mecanum Drive")
abstract public class KaitlinTestMecanumDrive extends MasterTeleOp {

    void mecanumDrive () {

        double x;
        double y;
        double pivotPower;

        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        pivotPower = gamepad1.right_stick_x;

        powerFL = y + x + pivotPower;
        powerBL = y - x + pivotPower;
        powerFR = y - x - pivotPower;
        powerBR = y + x - pivotPower;

    }

}
