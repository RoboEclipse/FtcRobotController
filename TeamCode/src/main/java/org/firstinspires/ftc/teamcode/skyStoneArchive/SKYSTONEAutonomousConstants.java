package org.firstinspires.ftc.teamcode.skyStoneArchive;


import com.acmerobotics.dashboard.config.Config;

@Config
public class SKYSTONEAutonomousConstants {

    //Speeds
    static double maxSpeed = 57;

    //Distances:

    //Foundation Constants
    public static double aFoundationAim = -10;
    public static double bFoundationDistance = -35;
    public static double cFoundationTurn = 105;
    public static double dWallStrafe = 23;
    public static double eSkybridge1 = 20;
    public static double eSkybridge2 = 20;
    public static double cFoundationClearPart1 = 40;
    public static double cFoundationClearPart2 = 20;
    public static double eSkybridge = 10;
    //New Mixed Constants
    public static double driveToStoneDistance = 30;
    public static double pullStoneOutDistance = -5;
    public static double driveUnderBridgeDistance = 75;
    public static double hitFoundationDistance = -10;
    public static double dropOffStoneDistance = 15;
    public static double parkDistance = 40;
    public static double moveAwayFromStoneDistance = -10;
    public static double foundationAlign = 24;
    public static double foundationDistance = -13;
    public static double skystoneClear = 12;
    public static int frontClawsWaitLength = 300;
    public static double fieldSize = 144;
    //Double Constants
    public static double doubleSkyStoneDistance1 = 5;
    public static double doubleAdjustDistance = 9.5;
    public static double doubleBridgeCross = 47;
    public static double doubleWallDistance = 8;
    public static double doubleCenterDistance = 9;
    //Shared Constants
    public static double shiftDistance = 7.5;
    //PID loop constants, in inches
    public static double reducePowerDistance = 30;
    public static double flooringPower = 0.3;

    //Front Side base servo
    public static double fbReady = 0.37;
    public static double fbDown = 0.20;
    public static double fbUp = 0.66;
    public static double fbStartPosition = 0.85;
    //Front Side Autonomous Claw
    public static double fsOpen = 0.15;
    public static double fsReady = 0.21;
    public static double fsGrab = 0.45;
    //Back Side base servo
    public static double bbReady = 0.86;
    public static double bbDown = 0.95;
    public static double bbUp = 0.54;
    public static double bbStartPosition = 0.4;
    //Back Side Autonomous Claw
    public static double bsOpen = 0.46;
    public static double bsReady = 0.35;
    public static double bsGrab = 0.12;

    //Distance sensor drive constants
    public static double kR = 1.0/25;
    public static double kD = 0.004;
    public static double min = 0.5;
    //Distance sensor PID
    public static double ddkR = -0.01;
    public static double ddkP = 0.08;
    public static double ddkD = -0.016;
    //Encoder PID
    public static double ekR = -0.01;
    public static double ekP = 0.16;
    public static double ekD = -0.025;
    public static double minimumPower = 0.2;
    public static double lateralFactor = 1.23;
    //Encoder drive constants
    //Coordinate Constants
    //Adjust in autonomous run
    public static double stoneGrabXBlue = 31.5;
    public static double stoneGrabXRed = SKYSTONEAutonomousConstants.fieldSize - stoneGrabXBlue;
    //Adjust to detect using sensorTest
    public static double stoneDetectXBlue = stoneGrabXBlue - 1;
    public static double stoneDetectXRed = fieldSize - stoneDetectXBlue;
    public static double stoneCrossXBlue = stoneDetectXBlue - 10;
    public static double stoneCrossXRed = fieldSize - stoneCrossXBlue;
    public static double stoneGrabY = 4;
    //
    public static double stoneAwayXBlue = 23;
    public static double stoneAwayXRed = SKYSTONEAutonomousConstants.fieldSize - stoneAwayXBlue;
    public static double stoneAwayY = 40.0;
    //
    public static double stoneDropXBlue = 33;
    public static double stoneDropXRed = SKYSTONEAutonomousConstants.fieldSize - stoneDropXBlue;
    //
    public static double farStoneDropY = SKYSTONEAutonomousConstants.fieldSize - 3;
    public static double nearStoneDropY = SKYSTONEAutonomousConstants.fieldSize - 12;
    public static double firstBridgeCross = -45;
    public static double foundationClear = 11;
    public static double skyBridgeDrive = 30;
}
