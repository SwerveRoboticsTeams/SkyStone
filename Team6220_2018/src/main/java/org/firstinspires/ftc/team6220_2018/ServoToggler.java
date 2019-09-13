package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.hardware.Servo;

/*
    Handles back and forth servo motions that are common for manipulator servos.
*/

public class ServoToggler
{
    Servo servo;
    boolean isDeployed = false;
    double servoRetractedPosition;
    double servoDeployedPosition;

    public ServoToggler(Servo s, double retractedPosition, double deployedPostition)
    {
        servo = s;
        servoRetractedPosition = retractedPosition;
        servoDeployedPosition = deployedPostition;
        setToStartingPosition();
    }

    public void setToStartingPosition()
    {
        retract();
    }

    public void deploy ()
    {
        servo.setPosition(servoDeployedPosition);
        isDeployed = true;
    }

    /*
     necessary if the servo needs to be set to a specialized position, but we still want it to
     toggle correctly
    */
    public void deployToAlternatePosition (double position)
    {
        servo.setPosition(position);
        isDeployed = true;
    }

    public void retract ()
    {
        servo.setPosition(servoRetractedPosition);
        isDeployed = false;
    }

    public void toggle ()
    {
        if (isDeployed)
        {
            retract();
        }
        else
        {
            deploy();
        }
    }
}
