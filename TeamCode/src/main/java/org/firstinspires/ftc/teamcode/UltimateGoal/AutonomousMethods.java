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

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
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
    private ElapsedTime runtime = new ElapsedTime();

    public boolean opModeStatus(){
        return opModeIsActive();
    }



    // Initializations
    public void initializeAutonomousDrivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
    }

    public void initializeAutonomousAttachments(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initialize(hardwareMap, telemetry);
    }



    // Game specific stuff (NEEDS ATTACHMENTS)
    public void shootRings (double shooterPower) {
        myRobot.elevatorServo.setPosition(Constants.elevatorTop);
        myRobot.tiltServo.setPosition(Constants.topTilt);
        //TODO: Testing negative shooter power remove later
        myRobot.shooterMotor.setPower(-shooterPower);
        sleep(3600);
        for (int i = 0; i < 4; i++) {
            myRobot.ringPushServo.setPosition(Constants.ringPush);
            sleep(360);
            myRobot.ringPushServo.setPosition(Constants.ringPushBack);
            sleep(1008);
        }
        myRobot.shooterMotor.setPower(0);
        myRobot.elevatorServo.setPosition(Constants.elevatorBottom);
        myRobot.tiltServo.setPosition(Constants.bottomTilt);
    }

    public void setCollectorPower(int collectorPower) {
        myRobot.collectionMotor.setPower(collectorPower);
    }

    public void setShooterAngle(double position) {
        myRobot.shooterTiltServo.setPosition(position);
    }

    public void raiseWobble() {
        setWobbleMotorPosition(0.9, Constants.wobbleTop);
    }

    public void lowerWobble() {
        setWobbleMotorPosition(0.9, Constants.wobbleBottom);
    }

    public void hoverWobble() {
        setWobbleMotorPosition(0.9, Constants.wobbleHover);
    }


//    public void grabWobble() {  // Grabs and raises wobble arm
//        myRobot.wobbleGoalServo.setPosition(Constants.wobbleClose);
//        sleep(250);
//        setWobbleMotorPosition(0.9, true);
//    }
//
//    public void lowerWobble() { // Lowers arm and release wobble claw
//        setWobbleMotorPosition(0.9, false);
//        myRobot.wobbleGoalServo.setPosition(Constants.wobbleOpen);
//    }
//
//    public void dropWobble() { //  Lowers arm, releases wobble claw, and raises wobble claw back up
//        lowerWobble();
//        setWobbleMotorPosition(0.8, true);
//    }

    public void setWobbleMotorPower(double power) {
        myRobot.runWobbleMotor(power);
    }

    public void setWobbleMotorPosition(double speed, int position){
        myRobot.wobbleGoalMotor.setTargetPosition(position);
        myRobot.wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myRobot.wobbleGoalMotor.setPower(speed);
    }

    public void setWobbleClaw(boolean wantClose){
        if (wantClose){
            myRobot.setWobbleClaw(Constants.wobbleClose);
        } else {
            myRobot.setWobbleClaw(Constants.wobbleOpen);
        }
    }

    public void setWobbleMotor(double speed, boolean goingUp, double timeoutS){
        int newTarget;
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            if (goingUp){
                newTarget = Constants.wobbleTop;
            } else {
                newTarget = Constants.wobbleBottom;
            }
            myRobot.wobbleGoalMotor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            myRobot.wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            myRobot.wobbleGoalMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (myRobot.wobbleGoalMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newTarget);
                telemetry.addData("Path2",  "Running at %7d",
                        myRobot.wobbleGoalMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            myRobot.wobbleGoalMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            myRobot.wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
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
                Log.d("Sample Label", detection);
            } else {
                detection = "None";
                telemetry.addData("Sample Label", "Nothing detected");
                Log.d("Sample Label", "Nothing detected");
            }
            telemetry.update();
        }

        int wobbleDropx = 0;
        int wobbleDropy = 0;
        if (!isRed) {
            if (detection.equals("Quad")) {
                wobbleDropx = 36;
                wobbleDropy = 54;
            } else if (detection.equals("Single")) {
                wobbleDropx = 18;
                wobbleDropy = 30;
            } else {
                wobbleDropx = 0;
                wobbleDropy = 54;
            }
        } else {
            if (detection.equals("Quad")) {
                wobbleDropx = 48;
                wobbleDropy = -54;
            } else if (detection.equals("Single")) {
                wobbleDropx = 24;
                wobbleDropy = -30;
            } else {
                wobbleDropx = 0;
                wobbleDropy = -54;
            }
        }
        return new Pose2d(wobbleDropx, wobbleDropy, Math.toRadians(0));
    }



    // Drive stuff
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



    // IMU Stuff
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



    // Tensorflow detection stuff (NEEDS CAMERA)
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AZU6IGT/////AAABmejCcqZy6k3el50rc42EGZ12WUI69Jz0IiavcLq/8JjxlUITrNJkegwqzTp6kR52Z+03jWcxJNGGH4dvST5xv+KXIlN/USxGznxymvRbPq6jVZych8Sp1YUVrWtin3C/qBwy5tLTs8uMRR2gT/zQfSM+AJXinLiuVTLxCas4XFfdErJdz43PDk8eoWzNYvhOM5S31HotPyZg411rgSyMNRcmzE1x2iAPRSN6JCqbDxKqBfk6m51QW0krJsWy2ECF2Ue3XI8Ro0yA3JoUo+PhJ29jRsYcCliDnydTnYQvZYwrFU0DPUFAfJHhPI4SEy4kFxuKWvYgQpUTFdiFxHHRX94Xei6GbRp6A5H4gmUfL2vz";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

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
}
