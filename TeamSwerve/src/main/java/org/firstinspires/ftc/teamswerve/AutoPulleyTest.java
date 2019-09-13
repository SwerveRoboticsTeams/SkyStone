package org.firstinspires.ftc.teamswerve;
import com.qualcomm.hardware.lynx.LynxCommExceptionHandler;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

// This opmode is used to test the new PID functionality using a weight suspended from the shaft
// of an Andymark 60

@Autonomous(name = "AutoPulley Test", group = "Autonomous")
public class AutoPulleyTest extends LinearOpMode
{
    DcMotorEx motor;

    // Allows us to change which mode the motor is running in each loop
    boolean isUsingRunToPosition = true;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize motor
        motor = (DcMotorEx)hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime timer = new ElapsedTime();
        double loopStartTime = 0;   // Takes snapshot of loop timer to find the time elapsed each loop
        double loopTime = 10;   // This is in ms


        waitForStart();
        timer.reset();


        // Main loop
        while (opModeIsActive())
        {
            loopStartTime = timer.milliseconds();

            // Make the motor's velocity oscillate back and forth.  Using sin(t) means that the
            // position of the motor will look like -cos(t)
            motor.setPower((Math.sin(timer.seconds())));

            while (timer.milliseconds() - loopStartTime < loopTime)
                idle();
        }
    }
}
