package org.firstinspires.ftc.teamswerve.ExampleModularInheritanceStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Mechanism class for a simple, 4-motor drivetrain.
 */
public class DriveTrain
{
    // Create drive motors:  Front Left, Front Right, Back Left, and Back Right.
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    // Initialize drive motors using hardware map given in MasterOpMode.
    public DriveTrain(HardwareMap hMap)
    {
        FL = hMap.dcMotor.get("motorFL");
        FR = hMap.dcMotor.get("motorFR");
        BL = hMap.dcMotor.get("motorBL");
        BR = hMap.dcMotor.get("motorBR");

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Generic drive method.  Others can be added.
    public void runDrive()
    {
        // Insert code for your drivetrain...
    }
}
