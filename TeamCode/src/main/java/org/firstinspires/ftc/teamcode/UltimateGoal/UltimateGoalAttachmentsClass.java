package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.baseBot.DrivetrainClass;
import org.firstinspires.ftc.teamcode.skyStoneArchive.SKYSTONEConstants;

public class UltimateGoalAttachmentsClass extends DrivetrainClass {
    private Telemetry telemetry;

    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //HardwareMaps
        initializeDriveTrain(hardwareMap, telemetry_);

    }
}
