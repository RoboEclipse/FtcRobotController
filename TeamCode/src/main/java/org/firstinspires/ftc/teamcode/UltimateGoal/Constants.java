package org.firstinspires.ftc.teamcode.UltimateGoal;

public class Constants {
    //TicksPerRotation is 42.78 on testbot
    private static double TICKS_PER_ROTATION = 103.6/3.7;
    private static double GEAR_RATIO = 16;
    private static double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    static double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4* Math.PI);

    //Speeds
    static double maxSpeed = 57;

    //Autonomous constants
    static int wobbleTop = 0;
    static int wobbleBottom = 0;

    //Tele-op constants
    static double wobbleHoldingPower = 0;
    static double wobbleRaisePower = 0;
    static double wobbleLowerPower = 0;
    static double wobbleClose = 0;
    static double wobbleOpen = 0;

    static double collectionPower = 0;

    static double ringPush = 0;
    static double ringPushBack = 0;

    static double setShooterAngle = 0;
    static double shooterAngleIncrease = 0.03;
    static double shooterPower = 0;

    static double elevatorTop = 0;
    static double elevatorBottom= 0;

    static double topTilt = 0;
    static double bottomTilt = 0;
}
