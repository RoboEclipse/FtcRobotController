package org.firstinspires.ftc.teamcode.skyStoneArchive;


import com.acmerobotics.dashboard.config.Config;

@Config
public class SKYSTONEConstants {

    //TicksPerRotation is 42.78 on testbot
    private static double TICKS_PER_ROTATION = 103.6/3.7;
    private static double GEAR_RATIO = 16;
    private static double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    static double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4* Math.PI);

    //Speeds
    static double maxSpeed = 57;

    public static int stoneDiff = 3;

    //Skystone Tele-op Stacking Claw Rotation Servo Constants
    public static double right90 = 1; //90 degrees right from straight
    public static double left90 = 0.39; //90 degrees left from straight
    public static double straight = 0.75; //Straight
    public static double oppositeSide = 0.00; //Make it straight but on the other side or back side
    //Skystone Tele-op Stacking Grabber Claw
    public static double tighten = 0.7;
    public static double loosen = 0.25;
    //Skystone Right Foundation Claw
    public static double rDown = 0.72;
    public static double rUp = 0.53;
    //Skystone Left Foundation Claw
    public static double lDown = 0.3; //At least .5
    public static double lUp = 0.55;

    public static double frontClawDown = 0.58;
    public static double frontClawUp = 1;
    //Capstone Servo
    public static double cUp = 0.28;
    public static double cDown = 0.6; //At least 0.55
    //Front right claw
    public static double frontLow = 0;
    public static double frontPlace = 0.45;
    public static double frontHigh = 1;
    public static double frontGrab = 0.0;
    public static double frontLoosen = 0.7;
    public static double frDown = 0.53;
    public static double frUp = 0.95;
    public static double frSuperUp = 1;
    //Front left claw
    public static double flDown = 0.78;
    public static double flUp = 0.3;
    public static double flSuperUp = 0.28;
    //Mode Switch Servo
    public static double deliverMode = 0.6;
    public static double stackingMode = 0.2;
}
