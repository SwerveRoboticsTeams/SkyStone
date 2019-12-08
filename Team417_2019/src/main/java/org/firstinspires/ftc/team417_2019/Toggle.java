package org.firstinspires.ftc.team417_2019;

public class Toggle {
    public boolean toggleState = false;
    public boolean prevState = false;
    public boolean press = false;

    public boolean getToggle(boolean button)
    {

        if(press)
        {
            toggleState = !toggleState;
        }

        if(! prevState && button)
        {
            press = true;
        }
        else if(press)
        {
            press = false;
        }
        prevState = button;

        return toggleState;

    }
}
