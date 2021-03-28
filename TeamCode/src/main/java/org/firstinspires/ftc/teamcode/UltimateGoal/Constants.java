package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //TicksPerRotation is 42.78 on testbot
    private static double TICKS_PER_ROTATION = 103.6/3.7;
    private static double GEAR_RATIO = 16;
    private static double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    static double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4* Math.PI);

    //Speeds
    static double maxSpeed = 57;



    //rest position(beginning setup)
    static int RESTPOS = 0;

    //grabber has to be angled during game to prevent from hitting elevator
    static int RUNPOS = 200;

    //lift wobble goal up to move over border of field
    static int VERTICALPOS = 700;

    //horizontal position where wobble goal being placed outside of field
    static int HORIZONTALPOS = 1360;

    //angled down to grab wobble goal
    static int GRABPOS = 1555;



    //Autonomous constants
    static int wobbleTop = 500;
    static int wobbleHover = 1400;
    static int wobbleBottom = 1600;


    //Tele-op constants
    static double wobbleHoldingPower = 0;
    static double wobbleRaisePower = 0.5;
    static double wobbleLowerPower = -0.5;
    public static double wobbleClose = 0.50;
    public static double wobbleOpen = 0.1;

    static double collectionPower = 1;

    static double ringPush = 0.8;
    static double ringPushBack = 0.55;

    public static double setShooterAngle = 0.5875030494738828; //0.500713393, .599
    public static double shooterPower = 0.53; //0.7
    public static double autoShooterPower = 0.54;

    static double elevatorTop = 0.3;
    static double elevatorBottom = 0.5;

    static double topTilt = 0.74;
    static double bottomTilt = 0.6;
}
