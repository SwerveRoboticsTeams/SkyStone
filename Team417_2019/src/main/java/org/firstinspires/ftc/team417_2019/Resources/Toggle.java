package org.firstinspires.ftc.team417_2019.Resources;

public class Toggle {
    public boolean toggleState = false;
    public boolean prevState = false;
    public boolean press = false;

    public boolean getToggle(boolean button)
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
        }
        // if the button is pressed (press is true) then toggle press to false because the button has been pushed
        else if(press)
        {
            press = false;
        }
        // set previous state to the current push of the button
        prevState = button;
        // return what the toggle state is
        return toggleState;

    }
}
