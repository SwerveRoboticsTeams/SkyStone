package org.firstinspires.ftc.teamswerve.ExampleModularInheritanceStructure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * The classes in this package offer an example of how to create a simple modular inheritance structure.
 * The robot is assumed to have a drivetrain with four motors and an arm with a motor and a servo.  For each
 * of these mechanisms, there exists a boolean that can be used to specify whether or not the mechanism is
 * attached during any given test of the robot.  If a mechanism has been removed, the boolean in question
 * can simply be changed to false and the code for the remaining attached mechanisms will still run.  However, do
 * note that robot's hardware must also be modular in order for this concept to work.
 */
abstract public class MasterOpMode extends LinearOpMode
{
    // Mechanism classes
    DriveTrain drive;
    Arm arm;

    // Booleans to specify whether a module is attached
    boolean isDriveAttached = true;
    boolean isArmAttached = true;

    public void initialize()
    {
        // Only initialize mechanisms if their respective booleans are true.
        if(isDriveAttached)
            drive = new DriveTrain(hardwareMap);
        if(isArmAttached)
            arm = new Arm(hardwareMap);
    }
}
