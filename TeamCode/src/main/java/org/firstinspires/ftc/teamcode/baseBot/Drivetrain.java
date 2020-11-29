package org.firstinspires.ftc.teamcode.baseBot;

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

public class Drivetrain {
    DcMotor lb, lf, rb, rf;
    ExpansionHubMotor lbBR, lfBR, rbBR, rfBR;
    ExpansionHubEx expansionHub;
    DistanceSensor leftDistance,backDistance, frontDistance;
    TestBotConfiguration names = new TestBotConfiguration();
    //Software
    private Telemetry telemetry;
    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    public void initializeDriveTrain(HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        lb = hardwareMap.dcMotor.get(names.leftBackMotor);
        lf = hardwareMap.dcMotor.get(names.leftFrontMotor);
        rb = hardwareMap.dcMotor.get(names.rightBackMotor);
        rf = hardwareMap.dcMotor.get(names.rightFrontMotor);
        //leftDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.leftDistance);
        //backDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backDistance);
        //frontDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.frontDistance);
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
    public String getOdometryWheels(){
        return "Front: " + lf.getCurrentPosition() + " Left: " + lb.getCurrentPosition() + " Right: " + rb.getCurrentPosition();
    }
    //Drive Stuff
    //Preferably Do Not Touch
    public void drive(double direction, double velocity, double rotationVelocity) {
        Wheels w = getWheels(direction, velocity, rotationVelocity);
        lf.setPower(w.lf);
        rf.setPower(w.rf);
        lb.setPower(w.lr);
        rb.setPower(w.rr);
        //telemetry.addData("Powers", String.format(Locale.US, "lf %.2f lr %.2f rf %.2f rr %.2f", w.lf, w.lr, w.rf, w.rr));
    }
    private static class Wheels {
        double lf, lr, rf, rr;

        Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }
    private Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        //final double vd = velocity;
        //final double td = direction;
        //final double vt = rotationVelocity;

        double s = Math.sin(direction + Math.PI / 4.0);
        double c = Math.cos(direction + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = velocity * s + rotationVelocity;
        final double v2 = velocity * c - rotationVelocity;
        final double v3 = velocity * c + rotationVelocity;
        final double v4 = velocity * s - rotationVelocity;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);

        return new Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }
    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    public double getAngle() {
        return imu.getAngularOrientation().firstAngle;
    }



    //Drivetrain
    void readEncoders(){
        telemetry.addData(
                "Encoders", "lf: " + lf.getCurrentPosition()
                        + " lb: " + lb.getCurrentPosition()
                        //+ " rf: " + rf.getCurrentPosition()
                        + " rb: "+ rb.getCurrentPosition()
        );
    }
}
