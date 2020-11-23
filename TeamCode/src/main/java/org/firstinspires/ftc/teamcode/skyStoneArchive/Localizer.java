package org.firstinspires.ftc.teamcode.skyStoneArchive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

public class Localizer {
    volatile double x = 0;
    volatile double y = 0;
    volatile double angle = 0;
    volatile double opticalX = 0;
    volatile double opticalY = 0;



    private SKYSTONEDrivetrainClass myRobot;
    private boolean encoder = false;
    private Corner corner = Corner.LEFT_DOWN;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    ElapsedTime clock;
    ArrayList<PiP> ALPIP;
    ArrayList<OCoord> ALOC;

    public Localizer(SKYSTONEDrivetrainClass input) {
        this.myRobot = input;
        clock = new ElapsedTime();
        ALPIP = new ArrayList<PiP>();
        ALOC = new ArrayList<OCoord>();
    }
    enum Corner {
        LEFT_UP,
        RIGHT_UP,
        LEFT_DOWN,
        RIGHT_DOWN
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getAngle(){
        return angle;
    }
    public void setCoordinates(double newX, double newY){
        x = newX;
        y = newY;
        updateCorner();
    }
    private void updateCorner(){
        if(x <SKYSTONEAutonomousConstants.fieldSize/2){
            if(y <SKYSTONEAutonomousConstants.fieldSize/2){
                corner = Corner.LEFT_DOWN;
            }
            else{
                corner = Corner.LEFT_UP;
            }
        } else {
            if(y <SKYSTONEAutonomousConstants.fieldSize/2){
                corner = Corner.RIGHT_DOWN;
            }
            else{
                corner = Corner.RIGHT_UP;
            }
        }
    }

    public Corner getCorner(){
        return corner;
    }


    public void update(RevBulkData prevData, RevBulkData encoderData){
        //Get positions
        int lfPosition = encoderData.getMotorCurrentPosition(myRobot.lf);
        int lbPosition = encoderData.getMotorCurrentPosition(myRobot.lb);
        int rfPosition = encoderData.getMotorCurrentPosition(myRobot.rf);
        int rbPosition = encoderData.getMotorCurrentPosition(myRobot.rb);
        //Get change in position
        int lfVelocity = encoderData.getMotorVelocity(myRobot.lf);
        int lbVelocity = encoderData.getMotorVelocity(myRobot.lb);
        int rfVelocity = encoderData.getMotorVelocity(myRobot.rf);
        int rbVelocity = encoderData.getMotorVelocity(myRobot.rb);
        //Translate encoder change to distance change
        double newDiffEncoderX = diffEncoderX(prevData, encoderData);
        double newDiffEncoderY = diffEncoderY(prevData, encoderData);
        //Estimate position of robot
        double encoderX = newDiffEncoderX + x;
        double encoderY = newDiffEncoderY + y;
        //Manual

        if (!Double.isNaN(encoderX)) {
            x = encoderX;
        }
        if (!Double.isNaN(encoderY)) {
            y = encoderY;
        }
        Log.d("Skystone:: ", "lf: " + lfPosition + " lb: " + lbPosition + " rf: " + rfPosition + " rb: " + rbPosition);
        /*
        if (!encoder) {
            //Calculate distance sensor change
            double newDiffOpticalX = getXRaw() - x;
            double newDiffOpticalY = getYRaw() - y;
            //Calculate preliminary error ratio for encoders
            double potentialXRatio = newDiffOpticalX/newDiffEncoderX;
            double potentialYRatio = newDiffOpticalY/newDiffEncoderY;
            //Guess at new x and y

            x = newDiffOpticalX + x;
            y = newDiffOpticalY + y;
            //Failsafe

            //Fall back to encoders if there's an issue
            double t1 = clock.nanoseconds();
            Log.d("Skystone:: ", "OpticalX: " + newDiffOpticalX +  " EncoderX: " + newDiffEncoderX
                    + " OpticalY: " + newDiffOpticalY + " EncoderY: " + newDiffEncoderY);
            //If readings and ratios are fine
            if(newDiffEncoderX != 0 && newDiffEncoderY != 0
                && newDiffOpticalX != 0 && newDiffOpticalY != 0
                && Math.abs(potentialXRatio)<3 && Math.abs(potentialYRatio)<3){
                //Add new value to PiP
                PiP value1 = new PiP(potentialXRatio, potentialYRatio, t1, encoderX, encoderY);
                ALPIP.add(value1);
            }
            //Limit PiP array size to 25
            if (ALPIP.size() >= 25){
                ALPIP.remove(0);
            }
        }
         */


        Log.d("Skystone: ", "Encoder Positions: lf: " + lfPosition + " lb: " + lbPosition +
                " rf: " + rfPosition + " rb: " + rbPosition);
        Log.d("Skystone: ", "Wheel Velocities: lf: " + lfVelocity + " lb: " + lbVelocity +
                " rf: " + rfVelocity + " rb: " + rbVelocity);
        Log.d("Skystone: ", "finalPosition: x: " + x + " y: " + y + " encoderX " + encoderX +
                " encoderY " + encoderY);
        updateCorner();
        dashboardGraphing();
    }

    private double diffEncoderX(RevBulkData prevData, RevBulkData curData){
        double multiplier = 1;
        if(corner == Corner.RIGHT_DOWN || corner == Corner.RIGHT_UP){
            multiplier = -1;
        }
        return (getTotalXPositions(curData)-getTotalXPositions(prevData))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier;

    }

    private double diffEncoderY(RevBulkData prevData, RevBulkData curData){
        double multiplier = 1;
        if(corner == Corner.RIGHT_DOWN || corner == Corner.RIGHT_UP){
            multiplier = -1;
        }
        return (getTotalYPositions(curData) - getTotalYPositions(prevData))
            /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier;
}

    private int getTotalXPositions(RevBulkData curData) {
        return curData.getMotorCurrentPosition(this.myRobot.lf)
                - curData.getMotorCurrentPosition(myRobot.lb)
                - curData.getMotorCurrentPosition(myRobot.rf)
                + curData.getMotorCurrentPosition(myRobot.rb);
    }

    private int getTotalYPositions(RevBulkData curData){
        return curData.getMotorCurrentPosition(myRobot.lf)
                + curData.getMotorCurrentPosition(myRobot.lb)
                + curData.getMotorCurrentPosition(myRobot.rf)
                + curData.getMotorCurrentPosition(myRobot.rb);
    }
    private double getXRaw() {
        synchronized (this) {
        }
        double xRaw = 0;
        switch (corner){
            case LEFT_DOWN: case LEFT_UP:
                xRaw = myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                if(xRaw>30){
                    xRaw = 400;
                }
                break;
            case RIGHT_DOWN: case RIGHT_UP:
                xRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                if(xRaw<SKYSTONEAutonomousConstants.fieldSize - 35){
                    xRaw = -400;
                }
                break;
        }
        return xRaw;
    }

    private double getYRaw() {
        double yRaw = 0;
        switch (corner){
            case LEFT_DOWN:
                yRaw = myRobot.backDistance.getDistance(DistanceUnit.INCH);
                break;
            case LEFT_UP:
                yRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.frontDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_DOWN:
                yRaw = myRobot.frontDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_UP:
                yRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.backDistance.getDistance(DistanceUnit.INCH);
                break;
        }
        return yRaw;
    }

    void updateOptical() {
        angle = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        switch (corner){
            case LEFT_DOWN: case LEFT_UP:
                opticalX = myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_DOWN: case RIGHT_UP:
                opticalX = SKYSTONEAutonomousConstants.fieldSize - myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                break;
        }
        if(Math.abs(opticalX-x) < 10){
            Log.d("Skystone", "Updated to OpticalX: " + opticalX + " PreviousX: " + x);
            x = opticalX;
        }
        double tx = clock.nanoseconds();
        switch (corner){
            case LEFT_DOWN:
                opticalY = myRobot.backDistance.getDistance(DistanceUnit.INCH);
                break;
            case LEFT_UP:
                opticalY = SKYSTONEAutonomousConstants.fieldSize - myRobot.frontDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_DOWN:
                opticalY = myRobot.frontDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_UP:
                opticalY = SKYSTONEAutonomousConstants.fieldSize - myRobot.backDistance.getDistance(DistanceUnit.INCH);
                break;
        }
        if(Math.abs(opticalY-y) < 10){
            Log.d("Skystone", "Updated to OpticalY: " + opticalY + " PreviousY: " + y);
            y = opticalY;
        }
        double ty = clock.nanoseconds();
        OCoord value2 = new OCoord(tx, ty, opticalX, opticalY, angle);
        synchronized (this) {
            ALOC.add(value2);

            if (ALOC.size() >= 25) {
                ALOC.remove(0);
            }
        }
    }



    public double getdY(RevBulkData bulkData){
        //Robot heading is flipped on red, so positive encoder = negative position
        int multiplier = 1;
        if(corner == Corner.RIGHT_DOWN || corner == Corner.RIGHT_UP){
            multiplier = -1;
        }
        return (bulkData.getMotorVelocity(myRobot.lf)
                + bulkData.getMotorVelocity(myRobot.lb)
                + bulkData.getMotorVelocity(myRobot.rf)
                + bulkData.getMotorVelocity(myRobot.rb))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier;
    }
    public double getdX(RevBulkData bulkData){
        //Robot heading is flipped on red, so positive encoder = negative position
        int multiplier = 1;
        if(corner == Corner.RIGHT_DOWN || corner == Corner.RIGHT_UP){
            multiplier = -1;
        }
        return (bulkData.getMotorVelocity(myRobot.lf)
                - bulkData.getMotorVelocity(myRobot.lb)
                - bulkData.getMotorVelocity(myRobot.rf)
                + bulkData.getMotorVelocity(myRobot.rb))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier;
    }

    public void useEncoderOnlyToggle(boolean useEncoderOnly){
        if (useEncoderOnly){
            encoder = true;
        } else if (!useEncoderOnly){
            encoder = false;
        }
    }
    boolean getEncoderOnly(){
        return encoder;
    }
    /*
    public void averageDiffs() {
        double xSum = 0;
        double ySum = 0;
        double size = ALPIP.size();

        if (size == 0)
        {
            // TODO log.
            return;
        }
        for (int i = 0; i < size; i++) {
            xSum = xSum + ALPIP.get(i).rX;
            ySum = ySum + ALPIP.get(i).rY;
        }
        strafeRatio = xSum / size;
        straightRatio = ySum / size;
        Log.d("Skystone:: ", "strafeRatio: " + strafeRatio + " straightRatio: " + straightRatio);
    }
    */

    public void dashboardGraphing(){
        if (dashboard != null){
            int apSize = ALPIP.size();
            int acSize = ALOC.size();
            double[] APIPX = new double[apSize];
            double[] APIPY = new double[apSize];
            double[] ALOCX = new double[acSize];
            double[] ALOCY = new double[acSize];
            if (apSize != 0) {
                packet.put("xRatio", ALPIP.get(apSize - 1).rX);
                packet.put("yRatio", ALPIP.get(apSize - 1).rY);
                for (int i = 0; i < apSize; i++) {
                    APIPX[i] = ALPIP.get(i).eX - 72;
                    APIPY[i] = ALPIP.get(i).eY - 72;
                }
            }
            if (ALOC.size() != 0) {
                for (int i = 0; i < acSize; i++) {
                    ALOCX[i] = - ALOC.get(i).oX + 72;
                    ALOCY[i] = ALOC.get(i).oY - 72;
                }
            }
            packet.put("x", getX());
            packet.put("y", getY());
            packet.fieldOverlay().setStroke("blue");
            packet.fieldOverlay().strokePolyline(APIPX, APIPY);
            packet.fieldOverlay().setStroke("red");
            packet.fieldOverlay().strokePolyline(ALOCX, ALOCY);
            dashboard.sendTelemetryPacket(packet);
        }
    }

}
