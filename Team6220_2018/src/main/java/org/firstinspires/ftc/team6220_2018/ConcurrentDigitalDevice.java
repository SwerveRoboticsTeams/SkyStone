package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ConcurrentDigitalDevice implements ConcurrentOperation
{
    private DigitalChannel channel;
    // Allows us to store past and present states of digital channel.
    boolean channelState;
    boolean lastChannelState;

    public ConcurrentDigitalDevice (DigitalChannel digitalChannel)
    {
        channel = digitalChannel;

        channelState = false;
        lastChannelState = false;
    }

    public boolean getState()
    {
        return channel.getState();
    }

    // Not in use.
    public void initialize(HardwareMap hMap){}

    // Call at end of loop
    public void update(double etime)
    {
        lastChannelState = channelState;

        channelState = channel.getState();
    }
}
