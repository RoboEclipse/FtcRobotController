package org.firstinspires.ftc.teamcode.skyStoneArchive;
//Point in Path
public class OCoord {
    double oXTS;
    double oYTS;
    double oX;
    double oY;
    double angle;

    public OCoord(double opticalXTS, double opticalYTS, double opticalX, double opticalY, double inputAngle){
        oXTS = opticalXTS;
        oYTS = opticalYTS;
        oX = opticalX;
        oY = opticalY;
        angle = inputAngle;
    }
}
