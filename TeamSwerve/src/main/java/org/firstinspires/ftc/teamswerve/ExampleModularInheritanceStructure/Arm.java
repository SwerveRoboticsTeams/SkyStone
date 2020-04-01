package org.firstinspires.ftc.teamswerve.ExampleModularInheritanceStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Mechanism class for a simple arm with one motor and one servo.
 */
public class Arm
{
    DcMotor armMotor;
    Servo armServo;

    // Initialize armMotor and armServo using hardware map given in MasterOpMode.
    public Arm(HardwareMap hMap)
    {
        armMotor = hMap.dcMotor.get("armMotor");
        armServo = hMap.servo.get("armServo");
    }

    // Generic arm movement method.  Others can be added.
    public void runArm()
    {
        // Insert code for your arm...
    }
}
