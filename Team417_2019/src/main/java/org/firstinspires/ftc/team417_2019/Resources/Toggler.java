package org.firstinspires.ftc.team417_2019.Resources;

import com.qualcomm.robotcore.hardware.Servo;

public class Toggler {
    public boolean toggleState = false;
    public boolean prevState = false;
    public boolean press = false;
    double in;
    double out;
    Servo servo;

    public Toggler(double in, double out, Servo servo) {
        this.in = in;
        this.out = out;
        this.servo = servo;
    }

    public void toggle(boolean button)
    {
        // if button is pressed then alternate the toggle state (this is the value returned to us)
        if(press)
        {
            toggleState = !toggleState;
        }
        // if prevState was false and the button is pushed we now know the button is pressed
        if(! prevState && button)
        {
            press = true;
            servo.setPosition(in);
        }
        // if the button is pressed (press is true) then toggle press to false because the button has been pushed
        else if(press)
        {
            press = false;
            servo.setPosition(out);
        }
        // set previous state to the current push of the button
        prevState = button;

    }

    public boolean getToggleState() {
        return toggleState;

    }
}
