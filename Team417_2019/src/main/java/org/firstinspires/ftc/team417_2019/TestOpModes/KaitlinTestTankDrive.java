package org.firstinspires.ftc.team417_2019.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417_2019.MasterTeleOp;

@Disabled
@TeleOp (name = "Test Tank Drive")
abstract public class KaitlinTestTankDrive extends MasterTeleOp
{

    void tankDrive() {
        double leftPower;
        double rightPower;

        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        double powerFL = leftPower;
        double powerBL = leftPower;
        double powerFR = rightPower;
        double powerBR = rightPower;
    }



}
