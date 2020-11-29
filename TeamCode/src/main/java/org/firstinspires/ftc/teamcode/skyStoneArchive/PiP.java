package org.firstinspires.ftc.teamcode.skyStoneArchive;
//Point in Path
public class PiP {
    double rX;
    double rY;
    double eX;
    double eY;
    double eTS;

    public PiP(double ratioX, double ratioY, double encoderTS, double encoderX, double encoderY){
        rX = ratioX;
        rY = ratioY;
        eTS = encoderTS;
        eX = encoderX;
        eY = encoderY;
    }
}
