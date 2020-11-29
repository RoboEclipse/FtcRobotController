package org.firstinspires.ftc.teamcode.skyStoneArchive;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

abstract class SKYSTONEAutonomousMethods extends AutonomousMethods {
    //Software
    //private Telemetry telemetry;

    private int turnPValue = 15;
    //Classes
    SKYSTONEClass skystoneClass(){
        //myRobot is from AutonomousMethods
        return (SKYSTONEClass)myRobot;
    }

    //Attachments
    //Backend
    void initializeSkystoneAutonomous(HardwareMap hardwareMap, Telemetry telemetry){
        initializeDrivetrainAutonomous(hardwareMap, telemetry, new SKYSTONEClass());
    }

    float getHue(ColorSensor sensor){
        float[] values = new float[3];
        int scale = 255;
        Color.RGBToHSV(sensor.red()*scale,
                sensor.green() * scale,
                sensor.blue() * scale, values
        );
        return values[0];
    }
    //For carrying the blocks
    void frontCarryStone () {
        skystoneClass().frontBase.setPosition(SKYSTONEAutonomousConstants.fbUp);
        skystoneClass().frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
    }
    void backCarryStone () {
        skystoneClass().backBase.setPosition(SKYSTONEAutonomousConstants.bbUp);
        skystoneClass().backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
    }
    //Basic grab
    void frontGrabStone (){
        skystoneClass().frontBase.setPosition(SKYSTONEAutonomousConstants.fbDown);
        sleep(250);
        skystoneClass().frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
        sleep(250);
    }
    void backGrabStone (){
        skystoneClass().backBase.setPosition(SKYSTONEAutonomousConstants.bbDown);
        sleep(250);
        skystoneClass().backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
        sleep(250);
    }
    //frontRelease is both the release and the starting position before grab
    void backReadyStone(){
        skystoneClass().backGrabber.setPosition(SKYSTONEAutonomousConstants.bsReady);
        skystoneClass().backBase.setPosition(SKYSTONEAutonomousConstants.bbReady);
    }
    //frontRelease is both the release and the starting position before grab
    void frontReadyStone(){
        skystoneClass().frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsReady);
        skystoneClass().frontBase.setPosition(SKYSTONEAutonomousConstants.fbReady);
    }
    void backReleaseStone(){
        skystoneClass().backBase.setPosition(SKYSTONEAutonomousConstants.bbReady);
        skystoneClass().backGrabber.setPosition(SKYSTONEAutonomousConstants.bsReady);
        sleep(300);
        skystoneClass().backBase.setPosition(SKYSTONEAutonomousConstants.bbStartPosition);
        sleep(100);

    }
    void frontReleaseStone(){
        skystoneClass().frontBase.setPosition(SKYSTONEAutonomousConstants.fbReady);
        skystoneClass().frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsReady);
        sleep(300);
        skystoneClass().frontBase.setPosition(SKYSTONEAutonomousConstants.fbStartPosition);
        sleep(100);
    }

    //VuforiaDetectionStuff

    //Vuforia Stuff

    //Complex Methods
    void getAngleWaitForStart() {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    telemetry.addData("Angle: ",  getHorizontalAngle() +  "");
                    telemetry.addData("Left Hue: ", getHue(skystoneClass().frontColor) + "");
                    telemetry.addData("Right Hue: ", getHue(skystoneClass().backColor) + "");
                    telemetry.addData("Left Distance: ", skystoneClass().getLeftDistance() + "");
                    telemetry.addData("Front Distance: ", skystoneClass().getFrontDistance() + "");
                    telemetry.addData("Back Distance: ", skystoneClass().getBackDistance()+ "");

                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }

    //(x1, y1): Coordinates of where you place the stone
    //(x2, y2): Coordinates of where you grab the next stone for depot case
    void placeAndGrab(double x1, double y1, double x2, double y2, String skyStonePosition, boolean isRedSide) {
        double adjustment = 0;
        int multiplier = 1;
        if(!isRedSide){
            multiplier = -1;
        }
        if(skyStonePosition.equals("Center")){
            adjustment = 8;
        }
        else if(skyStonePosition.equals("Bridge")){
            adjustment = 16;
        }
        Log.d("Skystone: ", "Skystoneposition " + skyStonePosition);
        //Cross bridge
        //localizer.useEncoderOnlyToggle(true);
        directionalDrive(x1+(7 * multiplier), y1 - 5, false, 2,0);
        //Get in position to place stone
        localizer.useEncoderOnlyToggle(false);
        directionalDrive(x1, y1, true, 2,0);
        //Place stone
        if(isRedSide) {
            frontReleaseStone();
            sleep(300);
            frontCarryStone();
        }
        else{
            backReleaseStone();
            sleep(300);
            backCarryStone();
        }
        //Prepare to cross bridge
        directionalDrive(x1+(7 * multiplier), y1 - 5, false, 2,0);
        int returnDistance = 70;
        //straighteningEncoderDriveNoStop(returnDistance*multiplier, 0, 50, 1);
        if(isRedSide){
            //Cross Bridge
            //localizer.useEncoderOnlyToggle(true);
            directionalDrive(x1+7*multiplier, y2 + adjustment - 10, false, 1,0);
            frontReadyStone();
            //Pick up stone
            localizer.useEncoderOnlyToggle(false);
            directionalDrive(x2, y2 + adjustment, true, 1,0);
            frontGrabStone();
        }
        else{
            backReadyStone();
            directionalDrive(x2, y2 + adjustment, true, 1,0);
            backGrabStone();
        }
        sleep(250);
        frontCarryStone();
        backCarryStone();
        if (isRedSide) {
            //Prepare to cross bridge
            directionalDrive(SKYSTONEAutonomousConstants.stoneAwayXRed, SKYSTONEAutonomousConstants.stoneAwayY, true, 2, 0);
        } else{
            directionalDrive(SKYSTONEAutonomousConstants.stoneAwayXBlue, SKYSTONEAutonomousConstants.stoneAwayY, true, 2, 0);
        }
        returnDistance = -50;
        //straighteningEncoderDriveNoStop(returnDistance*multiplier, 0, 50, 1);
    }
    void grabFoundation(boolean isRedSide) {
        int multiplier = 1;
        if(!isRedSide){
            multiplier = -1;
        }
        skystoneClass().leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        skystoneClass().rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        if(isRedSide){
            frontReleaseStone();
        }
        else{
            backReleaseStone();
        }


        encoderStrafeDriveInchesRight(-5,1);
        encoderTurn(90, 0.8, 2);
        //distanceEncoderDrive(38,1,0.8, 90, skystoneClass().frontDistance);
        runMotors(-0.4,-0.4);
        sleep(500);
        runMotors(0,0);
        skystoneClass().leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        skystoneClass().rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        sleep(250);
        if(isRedSide){
            //encoderStrafeDriveInchesRight(10,1);
            turn(41, 1, 0.4, 6);
            turn(0, 0, -1, 6);
            //encoderStraightDrive(-3, 1);
        }
        else{
            //encoderStrafeDriveInchesRight(-10,1);
            turn(141, 0.28,1,6);
            turn(178,-1, 0,6);
            //encoderStraightDrive(-3, 1);
        }
        //encoderStraightDrive(-12,1);
        skystoneClass().leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        skystoneClass().rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(250);
        encoderStraightDrive(7,1);
    }

    String detectSkyStonePosition(boolean isRedSide) {
        int multiplier = 1;
        double frontHue = getHue(skystoneClass().frontColor);
        double backHue = getHue(skystoneClass().backColor);
        String skyStonePosition = "Center";
        if(frontHue>70 && frontHue>backHue){
            if(isRedSide){
                skyStonePosition = "Depot";
            }
            else{
                skyStonePosition = "Bridge";
            }
        }
        else if(backHue>70 && backHue>frontHue){
            if(isRedSide){
                skyStonePosition = "Bridge";
            }
            else{
                skyStonePosition = "Depot";
            }

        }
        Log.d("Skystone: ", "FrontHue: " + frontHue + " BackHue: " + backHue);
        if(isRedSide){
            skystoneClass().backGrabber.setPosition(SKYSTONEAutonomousConstants.bsOpen);
            skystoneClass().backBase.setPosition(SKYSTONEAutonomousConstants.bbUp);
        }
        else{
            multiplier = -1;
        }
        if(skyStonePosition.equals("Center")){
            directionalDrive(SKYSTONEAutonomousConstants.stoneGrabXRed, SKYSTONEAutonomousConstants.stoneGrabY + 24 + 8,
                    true, 2, 0);
            //encoderStraightDrive(-8*multiplier, 1);
            //distanceEncoderDrive(baseDistance+8,0.5,1,0, skystoneClass().frontDistance);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+18.0, true, 2,0);
        }
        if(skyStonePosition.equals("Bridge")){
            directionalDrive(SKYSTONEAutonomousConstants.stoneGrabXRed, SKYSTONEAutonomousConstants.stoneGrabY + 24 + 16,
                    true, 2, 0);
            //encoderStraightDrive(-16*multiplier,1);
            //distanceEncoderDrive(baseDistance+16,0.5,1,0, skystoneClass().frontDistance);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+18.0, true, 2,0);
        }
        Log.d("Skystone: ", " skyStonePosition: " + skyStonePosition);
        telemetry.addData("SkystonePosition", skyStonePosition);
        telemetry.update();
        return skyStonePosition;
    }

    void park(boolean isRedSide){
        double skyBridgeDistance = SKYSTONEAutonomousConstants.skyBridgeDrive;
        resetClaws();
        int power = 1;
        int angle;
        if (isRedSide){
            angle = 0;
            encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.foundationClear, 1);
            encoderTurn(angle, 1, 2);
            /*
            runMotors(-.7,-.7);
            sleep(400);
            runMotors(0,0);
            skyBridgeDistance += 10;
            */
        } else{
            angle = 180;
            encoderStrafeDriveInchesRight(-SKYSTONEAutonomousConstants.foundationClear, 1);
            encoderTurn(angle, 1, 2);
        }
        encoderStraightDrive(skyBridgeDistance, 1);
    }
    void resetClaws(){
        skystoneClass().backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
        skystoneClass().frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
        skystoneClass().backBase.setPosition(SKYSTONEAutonomousConstants.bbStartPosition);
        skystoneClass().frontBase.setPosition(SKYSTONEAutonomousConstants.fbStartPosition);
    }
}