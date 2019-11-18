package org.firstinspires.ftc.team8923_2019;

public class Constants
{

    // Constants to be used in code. Measurements in millimeters
    static final double GEAR_RATIO = 1.0/1.0; // Ratio of driven gear to driving gear
    static final double TICKS_PER_MOTOR_REVOLUTION = 560.0;
    static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION / GEAR_RATIO;
    static final double WHEEL_DIAMETER = 4 * 25.4; // 4 inch diameter
    static final double MM_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;

    static final double MM_PER_TICK = MM_PER_REVOLUTION / TICKS_PER_WHEEL_REVOLUTION;
    static final double COUNTS_PER_MM = TICKS_PER_WHEEL_REVOLUTION / MM_PER_REVOLUTION;
    public static final double MINIMUM_JOYSTICK_PWR = 0.33;
    public static final double MINIMUM_TRIGGER_VALUE = 0.33;
    public static final double MINIMUM_DRIVE_POWER = 0.08;


    // The Arm


    public static final double MIN_ENCODERCOUNT_PARALLEL_POINT = -1360;
    public static final double MAX_ENCODERCOUNT_PARALLEL_POINT = -875;
    public static final double MIN_SERVOJOINT_PWR = 0.40;
    public static final double MAX_SERVOJOINT_PWR = 0.10;

    public static final double ROTATION_P = 0.005;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final double TRANSLATION_P = 0.001;
    public static final double TRANSLATION_I = 0.0;
    public static final double TRANSLATION_D = 0.0;
    public static final double ANGLE_TOLERANCE_DEG = 5.0;
    public static final double POSITION_TOLERANCE_MM = 20.0;

    public static double ARM_STARTING_TICKS;

}


