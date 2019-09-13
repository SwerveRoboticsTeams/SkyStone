package org.firstinspires.ftc.team6220_2019;

/*
     Used to store important constants for easy access in other classes.
*/

public class Constants
{
    // Standard conversions
     // This is for an Andymark 40; 20's and 60's are different
    public static final int  ANDYMARK_TICKS_PER_ROTATION = 1120;
    public static final int TETRIX_TICKS_PER_ROTATION = 1440;
    public static final float MM_PER_INCH = 25.4f;


    // Robot specifications
    public static final float WHEEL_DIAMETER_MM = 4 * MM_PER_INCH;    // 4 inch diameter wheel
    public static final double GEAR_RATIO = 24.0 / 32.0;            // Driven to driving gear
    // Actually 25.4 ticks / inch
    public static final double MM_PER_ANDYMARK_TICK = (Math.PI * WHEEL_DIAMETER_MM) / (ANDYMARK_TICKS_PER_ROTATION * GEAR_RATIO);
    public static final float WHEEL_SEPARATION_MM = 15.5f * MM_PER_INCH;    // 4 inch diameter wheel

    // Drive mode constants
    public static final double SLOW_MODE_T_FACTOR = 0.3;
    public static final double SLOW_MODE_R_FACTOR = 0.3;
    public static final double T_FACTOR = 1.0;
    public static final double R_FACTOR = 1.0;


    // Tolerances
    public static final double ANGLE_TOLERANCE_DEG = 2.5;
    public static final double POSITION_TOLERANCE_MM = 15.0;
    public static final double OPENCV_TOLERANCE_PIX = 0.5;


    // Movement control constants-----------------------
    public static final double MINIMUM_DRIVE_POWER = 0.15;   // todo Adjust
    public static final double MINIMUM_TURNING_POWER = 0.1;
     // Constants for adjusting powers that are proportional to angle and position differences
    public static final double TURNING_POWER_FACTOR = 0.02;
    public static final double DRIVE_POWER_FACTOR = 0.003;
    //------------------------------------------------


    // todo Adjust for this year's robot
    // PID loop constants-------------------------------
    public static final double ROTATION_P = TURNING_POWER_FACTOR;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.016;
    public static final double TRANSLATION_P = DRIVE_POWER_FACTOR;
    public static final double TRANSLATION_I = 0.00000002;    // todo Adjust
    public static final double TRANSLATION_D = 0.004;
    //---------------------------------------------------


    // Field specs
    public static final float MM_FIELD_SIZE = (12 * 12) * MM_PER_INCH;


    // Vuforia constants
    public static final int IMAGE_WIDTH = 1280;
    public static final int IMAGE_HEIGHT = 720;


    // Servo positions--------------------------------

    //-------------------------------------------------


    // Encoder positions-------------------------------

    //-------------------------------------------------


    // Ensure that input isn't used when no commands are given
    public static final double MINIMUM_JOYSTICK_POWER = 0.05;
    public static final double MINIMUM_TRIGGER_VALUE = 0.33;    // todo How large does this value need to be to prevent twitching?
}
