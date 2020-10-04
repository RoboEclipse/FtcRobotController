package org.firstinspires.ftc.teamcode.skyStoneArchive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

public class SKYSTONEDrivetrainClass {
    DcMotor lb, lf, rb, rf;
    ExpansionHubMotor lbBR, lfBR, rbBR, rfBR;
    ExpansionHubEx expansionHub;
    DistanceSensor leftDistance,backDistance, frontDistance;
    SKYSTONEConfiguration skystoneNames = new SKYSTONEConfiguration();
    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    void initializeDriveTrain(HardwareMap hardwareMap, Telemetry telemetry){
        lb = hardwareMap.dcMotor.get("backLeft");
        lf = hardwareMap.dcMotor.get("frontLeft");
        rb = hardwareMap.dcMotor.get("backRight");
        rf = hardwareMap.dcMotor.get("frontRight");
        leftDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.leftDistance);
        backDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backDistance);
        frontDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.frontDistance);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /* Bulk Read Styff
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        lbBR = (ExpansionHubMotor) hardwareMap.dcMotor.get("backLeft");
        lfBR = (ExpansionHubMotor) hardwareMap.dcMotor.get("frontLeft");
        rbBR = (ExpansionHubMotor) hardwareMap.dcMotor.get("backRight");
        rfBR = (ExpansionHubMotor) hardwareMap.dcMotor.get("frontRight");
        */
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setTargetPosition(300);
        lb.setPower(1);

        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    void runMotors (double leftPower, double rightPower){
        lb.setPower(leftPower);
        lf.setPower(leftPower);
        rb.setPower(rightPower);
        rf.setPower(rightPower);
    }
    double getBackDistance(){
        return backDistance.getDistance(DistanceUnit.INCH);
    }
    double getFrontDistance() {
        return frontDistance.getDistance(DistanceUnit.INCH);
    }
    double getLeftDistance() {
        return leftDistance.getDistance(DistanceUnit.INCH);
    }
}
