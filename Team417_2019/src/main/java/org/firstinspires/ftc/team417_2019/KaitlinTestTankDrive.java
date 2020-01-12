package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp (name = "Test Tank Drive")
abstract public class KaitlinTestTankDrive extends MasterTeleOp {

    void tankDrive() {
        double leftPower;
        double rightPower;

        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        powerFL = leftPower;
        powerBL = leftPower;
        powerFR = rightPower;
        powerBR = rightPower;
    }



}
