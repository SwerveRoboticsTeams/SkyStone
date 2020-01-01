package org.firstinspires.ftc.team6220_2019;

/*
     Used to store important constants for easy access in other classes.
*/

public class Constants
{
    // Standard conversions
    // This is for an AndyMark 40; 20s are 560 ticks / rot and 60s are 1680 ticks / rot
    public static final int AM_40_TICKS_PER_ROTATION = 1120;
    public static final float MM_PER_INCH = 25.4f;


    // Robot specifications
    public static final float WHEEL_DIAMETER_IN = 4;    // 4 inch diameter wheel
    public static final double SPROCKET_RATIO = 16.0 / 16.0;            // Driven to driving sprocket teeth
    public static final double GRAB_ARM_LENGTH = 13.5;                // Grabber arm is 13.5 inches long
    // todo Confirm this is correct
    public static final double IN_PER_ANDYMARK_TICK = (Math.PI * WHEEL_DIAMETER_IN) / (0.5 * AM_40_TICKS_PER_ROTATION * SPROCKET_RATIO);

    // Drive mode constants
    public static final double SLOW_MODE_T_FACTOR = 0.3;
    public static final double SLOW_MODE_R_FACTOR = 0.3;
    public static final double T_FACTOR = 1.0;
    public static final double R_FACTOR = 1.0;
    public static final double MAX_NAV_ROT_POWER = 0.3;
    public static final double AUTO_SEARCH_TURN_POWER = 0.2;

    public static final double LIFT_POWER_FACTOR = 0.6;
     // Constants to control power of collector motors
    public static final double COLLECTOR_POWER = 0.2;
    public static final double COLLECTOR_ROTATE_POWER = 0.3;    // todo Adjust

    // Constants to control slide motors
    public static final double SLIDE_MOTOR_MAX_POWER = 0.8;    // todo once we know this is working, increase to 1.0
    public static final double SLIDE_MOTOR_LOW_POWER_FACTOR = 0.2;
    public static final int SLIDE_MOTOR_MIN_DIST = 0;
    public static final int SLIDE_MOTOR_MAX_DIST = 3000;
    public static final int SLIDE_EXTENDED = 1300;     // todo Adjust

    // Autonomous Drive constants
    public static final float AUTONOMOUS_SCALE_DISTANCE = 24;
    public static final float AUTONOMOUS_SCALE_ANGLE = 90;


    // Tolerances
    public static final double ANGLE_TOLERANCE_DEG = /*1.0*/3.0;
    public static final double POSITION_TOLERANCE_IN = /*0.2*/1.0;
    public static final double OPENCV_TOLERANCE_PIX = 0.5;
    public static final double LIFT_MOTOR_TOLERANCE_ENC_TICKS = 20;


    // Movement control constants-----------------------
    public static final double MINIMUM_DRIVE_POWER = 0.08;   // todo Adjust
    public static final double MAX_DRIVE_POWER = 1.0;   // todo Adjust
    public static final double MINIMUM_TURNING_POWER = 0.02;
    // Constants for adjusting powers that are proportional to angle and position differences
    public static final double TURNING_POWER_FACTOR = 0.01;
    public static final double DRIVE_POWER_FACTOR = 0.04;
    //------------------------------------------------


    // todo Implement I and D terms
    // PID loop constants-------------------------------
    public static final double ROTATION_P = TURNING_POWER_FACTOR;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final double TRANSLATION_P = DRIVE_POWER_FACTOR;
    public static final double TRANSLATION_I = 0/*0.0000005*/;
    public static final double TRANSLATION_D = 0/*0.1*/;
    //---------------------------------------------------


    // Field specs
    public static final float MM_FIELD_SIZE = (12 * 12) * MM_PER_INCH;


    // Vuforia constants
    public static final int IMAGE_WIDTH = 1280;
    public static final int IMAGE_HEIGHT = 720;
    public static final double WEBCAM_1_OFFSET = 180.0;    // Need to account for fact that webcam faces opposite direction of drivetrain.
    public static final int WEBCAM_2_OFFSET = 0;    // * 2nd webcam not currently in use.


    // Servo positions--------------------------------
     // For the grabber, 1 is closed, 0 is open. todo adjust grabber constants for new grabber
    public static final double GRABBER_OPEN = 0.65;
    public static final double GRABBER_CLOSED = 0.9;
    public static final double PARALLEL_SERVO_INIT = 0.0;
     // Open = stowed, closed = grabbing foundation
    public static final double FOUNDATION_SERVO_LEFT_OPEN = 0;
    public static final double FOUNDATION_SERVO_LEFT_CLOSED = 1.0;
    public static final double FOUNDATION_SERVO_RIGHT_OPEN = 1.0;
    public static final double FOUNDATION_SERVO_RIGHT_CLOSED = 0;

    //todo adjust grabberArm constants
    public static final double GRABBER_ARM_SERVO_LEFT_EXTEND = 1.0;
    public static final double GRABBER_ARM_SERVO_LEFT_RETRACT = 0.0;
    public static final double GRABBER_ARM_SERVO_RIGHT_EXTEND = 0.0;
    public static final double GRABBER_ARM_SERVO_RIGHT_RETRACT = 1.0;

    // This converts encoder ticks to REV smart servo (in 225 degree mode) positions
    public static final double MOTOR_TO_REV_SERVO_MOVEMENT = 1.55;
    //-------------------------------------------------


    // Encoder positions-------------------------------
    public static final double LIFT_MOTOR_TICKS = 2.0 * 1680.0;
    public static final int LIFT_GRAB_POS = 55;
    public static final int LIFT_PLACE_POS = 2000;
    //-------------------------------------------------

    // Other encoder constants
    public static final double DRIVE_POWER_TO_ENCODER_PROPORTION = 1; //todo Adjust


    // Ensure that input isn't used when no commands are given
    public static final double MINIMUM_JOYSTICK_POWER = 0.05;
    public static final double MINIMUM_TRIGGER_VALUE = 0.05;    // todo Adjust
}
