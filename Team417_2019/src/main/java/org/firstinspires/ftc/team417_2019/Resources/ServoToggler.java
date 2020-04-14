package org.firstinspires.ftc.team417_2019.Resources;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoToggler {
    public Toggler toggle = new Toggler();
    double in;
    double out;
    private Servo servo;

    public ServoToggler(double in, double out, Servo servo) {
        this.in = in;
        this.out = out;
        this.servo = servo;
    }

    public void toggle(boolean button)
    {
        if (toggle.toggle(button)) {
            servo.setPosition(in);
        }
        else {
            servo.setPosition(out);
        }
    }

    public boolean getToggleState() {
        return toggle.getToggleState();
    }
}
