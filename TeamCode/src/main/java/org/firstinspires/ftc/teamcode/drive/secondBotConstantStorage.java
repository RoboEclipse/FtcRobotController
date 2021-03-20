package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.config.Config;

@Config
public class secondBotConstantStorage {
    //Competition Robot
    //private boolean cFrontWheelReversed = false;

    //Test Robot
    //DriveConstants
    //SampleMecanumDrive
    //StandardTrackingWheelLocalizer
    public static double LATERAL_DISTANCE = 13.25;
    public static double FORWARD_OFFSET = -4;
    public static double trackWidth = 11;
    public static double kV = 0.02061;
    public static double kA = 0.00002;
    public static double kStatic = 0.045;
    public static int TicksPerRev = 1440;
    public static boolean FrontWheelReversed = true;
    public static double MaxRPM = 800;
    public static double wheelRadius = 0.75;
    public static double gearRatio = 1;
    public static double TkP;
    public static double TkI;
    public static double TkD;
    public static double HkP;
    public static double HkI;
    public static double HkD;
}
