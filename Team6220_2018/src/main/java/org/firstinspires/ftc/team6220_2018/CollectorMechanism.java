package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CollectorMechanism implements ConcurrentOperation
{
    MasterOpMode master;
    DriverInput driverInput;
    CRServo collector;
    ConcurrentDigitalDevice channel;
    ElapsedTime timer;

    // Collector operation booleans
    boolean collectorSlowMode = false;
    boolean isCollectorStopping = false;
    boolean isCollecting = false;
    boolean isCollectingIn = false;
    boolean isFastEjecting = false;
    // Allows us to break out of collector encoder loop if necessary.
    ElapsedTime collectorLoopTimer = new ElapsedTime();

    // Allows us to count how many positions the encoder has passed.
    int encoderCount = 0;

    double collectorPowerIn = Constants.MOTOR_COLLECTOR_IN;
    double collectorPowerOut = Constants.MOTOR_COLLECTOR_OUT;

    // Construct class with necessary outside objects.
    public CollectorMechanism(MasterOpMode masterOpMode, DriverInput driver2, CRServo motorCollector, ConcurrentDigitalDevice collectorChannel, ElapsedTime collectorLoopTimer)
    {
        master = masterOpMode;
        collector = motorCollector;
        channel = collectorChannel;
        timer = collectorLoopTimer;
        driverInput = driver2;
    }

    // Not in use.
    public void initialize(HardwareMap hMap){}


    // Uses driver 2 input to drive arm and collector motors.
    // Call at end of loop.
    public void update(double etime)
    {

        if (driverInput.isButtonJustPressed(Button.RIGHT_BUMPER) && !collectorSlowMode)
        {
            collectorPowerIn = Constants.MOTOR_COLLECTOR_SLOW_IN;
            collectorSlowMode = true;
        }
        else if (driverInput.isButtonJustPressed(Button.RIGHT_BUMPER) && collectorSlowMode)
        {
            collectorPowerIn = Constants.MOTOR_COLLECTOR_IN;
            collectorSlowMode = false;
        }


        /*
         * Collect and eject minerals; collecting can be in normal or slow mode while ejecting is
         * always slow.  Buttons have to be held to power collector.
         */
        if (driverInput.isButtonPressed(Button.DPAD_DOWN))
        {
            collector.setPower(collectorPowerIn);
            isCollectingIn = true;
        }
        else if (driverInput.isButtonPressed(Button.DPAD_UP))
        {
            collector.setPower(Constants.MOTOR_COLLECTOR_SLOW_OUT);
            isCollectingIn = false;
        }
        // Allows rapid removal of minerals.
        else if (driverInput.isButtonPressed(Button.LEFT_BUMPER))
        {
            collector.setPower(Constants.MOTOR_COLLECTOR_OUT);
            isCollectingIn = false;
        }
        else
        {
            // Run motor in slow mode while it is approaching encoder locations.
            if ((Math.abs(collector.getPower()) > 0.01) && isCollectingIn)
            {
                collector.setPower(Constants.MOTOR_COLLECTOR_SLOW_IN);
            }
            else if ((Math.abs(collector.getPower()) > 0.01) && !isCollectingIn)
            {
                collector.setPower(Constants.MOTOR_COLLECTOR_SLOW_OUT);
            }

            timer.reset();

            // Wait until optical encoder moves 2 positions (this gives it extra time to slow down
            // if it was going full speed).  Only continue to power motor if timer reads shorter
            // than 2 seconds since we do not want the collector to accidentally stay powered forever.
            if (channel.channelState || (timer.seconds() > 2))
                collector.setPower(0);

            master.telemetry.addData("Collector Channel: ", channel.channelState);
            master.telemetry.update();
        }
    }
}
