package org.firstinspires.ftc.teamswerve.ExampleModularInheritanceStructure;

/**
 * This class runs the modular code implemented in MasterOpMode and the mechanism classes.
 */
public class OpMode extends MasterOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        waitForStart();

        // Only run mechanism-specific methods if their respective modules are attached.
        while(opModeIsActive())
        {
            if(isDriveAttached)
                drive.runDrive();
            if(isArmAttached)
                arm.runArm();
            idle();
        }
    }
}
