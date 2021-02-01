package org.firstinspires.ftc.teamcode.UltimateGoal;


import android.util.Log;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.baseBot.Drivetrain;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.RevBulkData;

import java.util.List;

abstract public class AutonomousMethods extends LinearOpMode {
    Attachments myRobot = new Attachments();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    private Orientation angles;

    //Tensorflow detection stuff
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AZU6IGT/////AAABmejCcqZy6k3el50rc42EGZ12WUI69Jz0IiavcLq/8JjxlUITrNJkegwqzTp6kR52Z+03jWcxJNGGH4dvST5xv+KXIlN/USxGznxymvRbPq6jVZych8Sp1YUVrWtin3C/qBwy5tLTs8uMRR2gT/zQfSM+AJXinLiuVTLxCas4XFfdErJdz43PDk8eoWzNYvhOM5S31HotPyZg411rgSyMNRcmzE1x2iAPRSN6JCqbDxKqBfk6m51QW0krJsWy2ECF2Ue3XI8Ro0yA3JoUo+PhJ29jRsYcCliDnydTnYQvZYwrFU0DPUFAfJHhPI4SEy4kFxuKWvYgQpUTFdiFxHHRX94Xei6GbRp6A5H4gmUfL2vz";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public void initializeAutonomousDrivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
    }

    public void initializeAutonomousAttachments(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initialize(hardwareMap, telemetry);
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    @NotNull
    public Pose2d getWobbleDropPose(boolean isRed) {
        //Detection happens here
        String detection = "";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            String label = "";
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if ((updatedRecognitions != null) && (updatedRecognitions.size() > 0)) {
                detection = updatedRecognitions.get(0).getLabel();
                telemetry.addData("Sample Label", detection);
            } else {
                detection = "none";
                telemetry.addData("Sample Label", "Nothing detected");
            }
            telemetry.update();
        }

        int wobbleDropx = 0;
        int wobbleDropy = 0;
        if (!isRed) {
            if (detection.equals("Quad")) {
                wobbleDropx = 60;
                wobbleDropy = 60;
            } else if (detection.equals("Single")) {
                wobbleDropx = 36;
                wobbleDropy = 36;
            } else {
                wobbleDropx = 12;
                wobbleDropy = 60;
            }
        } else {
            if (detection.equals("Quad")) {
                wobbleDropx = 60;
                wobbleDropy = -60;
            } else if (detection.equals("Single")) {
                wobbleDropx = 36;
                wobbleDropy = -36;
            } else {
                wobbleDropx = 12;
                wobbleDropy = -60;
            }
        }
        return new Pose2d(wobbleDropx, wobbleDropy, Math.toRadians(0));
    }

    public void setModeAllDrive(DcMotor.RunMode mode){
        myRobot.lb.setMode(mode);
        myRobot.lf.setMode(mode);
        myRobot.rb.setMode(mode);
        myRobot.rf.setMode(mode);
    }

    public void runMotors (double leftPower, double rightPower){
        myRobot.lb.setPower(leftPower);
        myRobot.lf.setPower(leftPower);
        myRobot.rb.setPower(rightPower);
        myRobot.rf.setPower(rightPower);
    }

    private void multiSetTargetPosition(double ticks, DcMotor...motors){
        for(DcMotor motor:motors){
            motor.setTargetPosition((int) Math.round(ticks));
        }
    }
    private boolean notCloseEnough(int tolerance, DcMotor...motors){
        for(DcMotor motor : motors){
            if(Math.abs(motor.getCurrentPosition()-motor.getTargetPosition()) > tolerance){
                return true;
            }
        }
        return false;
    }

    public void encoderStraightDrive(double inches, double power){
        encoderStraightDriveNoStop(inches, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderStraightDriveNoStop(double inches, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStraightDriveNoReset(inches, power);
    }

    public void encoderStraightDriveNoReset(double inches, double power){
        ElapsedTime time = new ElapsedTime();
        multiSetTargetPosition(inches* Constants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && time.milliseconds()<4000 && opModeIsActive()){
            Log.d("Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
    }

    //Negative = Left, Positive = Right
    public void encoderStrafeDriveInchesRight(double inches, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.lf.setTargetPosition((int) Math.round(inches*Constants.TICKS_PER_INCH));
        myRobot.lb.setTargetPosition(-(int) Math.round(inches*Constants.TICKS_PER_INCH));
        myRobot.rf.setTargetPosition(-(int) Math.round(inches*Constants.TICKS_PER_INCH));
        myRobot.rb.setTargetPosition((int) Math.round(inches*Constants.TICKS_PER_INCH));
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        ElapsedTime killTimer = new ElapsedTime();
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.lb, myRobot.rf, myRobot.rb) && opModeIsActive() && killTimer.seconds()<2){
            Log.d("SkyStone Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("SkyStone Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("SkyStone Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("SkyStone Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turn(double targetAngle, double leftPower, double rightPower, double tolerance){
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        ElapsedTime killTimer = new ElapsedTime();
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        while(Math.abs(error)>tolerance && opModeIsActive() && killTimer.seconds()<3){
            currentAngle = getHorizontalAngle();
            error = targetAngle-currentAngle;
            error = loopAround(error);
            runMotors(leftPower, rightPower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + leftPower + "rightPower: " + rightPower + "CurrentAngle: " + currentAngle);
        }
    }

    //IMU Stuff
    public double getHorizontalAngle(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.firstAngle;
        output = loopAround(output);
        return output;
    }

    private double loopAround(double output) {
        if (output > 180) {
            output -= 360;
        }
        if (output < -180) {
            output += 360;
        }
        return output;
    }

    public double getRoll(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.secondAngle;
        output = loopAround(output);
        return output;
    }

    public double getVerticalAngle(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.thirdAngle;
        output = loopAround(output);
        return output;
    }

    public boolean opModeStatus(){
        return opModeIsActive();
    }

}
