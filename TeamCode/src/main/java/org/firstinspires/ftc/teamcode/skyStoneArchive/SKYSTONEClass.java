package org.firstinspires.ftc.teamcode.skyStoneArchive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;
import java.util.Locale;

import static java.lang.Thread.sleep;


public class SKYSTONEClass extends SKYSTONEDrivetrainClass{
    //Hardware
    ColorSensor frontColor, backColor;
    DcMotor clawSlide, elevator, rightCollectorMotor, leftCollectorMotor;
    Servo clawRotation, leftFoundationServo, rightFoundationServo, clawServo, frontBase, frontGrabber, capServo, backBase, backGrabber, modeSwitch;
    DigitalChannel limitSwitch;
    //Software
    private Telemetry telemetry;

    //Classes

    //SKYSTONEDrivetrainClass drivetrain = new SKYSTONEDrivetrainClass();
    //private SKYSTONEConstants skystoneConstants = new SKYSTONEConstants();

    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //HardwareMaps
        initializeDriveTrain(hardwareMap, telemetry_);
        /*
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        */
        limitSwitch = hardwareMap.digitalChannel.get(skystoneNames.limitSwitch);
        frontColor = hardwareMap.colorSensor.get(skystoneNames.frontColor);
        backColor = hardwareMap.colorSensor.get(skystoneNames.backColor);
        clawRotation = hardwareMap.servo.get(skystoneNames.rotationServo);
        clawSlide = hardwareMap.dcMotor.get(skystoneNames.slidingMotor);
        elevator = hardwareMap.dcMotor.get(skystoneNames.elevator);
        leftFoundationServo = hardwareMap.servo.get(skystoneNames.leftFoundationServo);
        rightFoundationServo = hardwareMap.servo.get(skystoneNames.rightFoundationServo);
        capServo = hardwareMap.servo.get(skystoneNames.cappingServo);
        clawServo = hardwareMap.servo.get(skystoneNames.collectorServo);
        frontBase = hardwareMap.servo.get(skystoneNames.frontBase);
        frontGrabber = hardwareMap.servo.get(skystoneNames.frontGrabber);
        backBase = hardwareMap.servo.get(skystoneNames.backBase);
        backGrabber = hardwareMap.servo.get(skystoneNames.backGrabber);
        backDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backDistance);
        leftCollectorMotor = hardwareMap.dcMotor.get(skystoneNames.leftCollectorMotor);
        rightCollectorMotor = hardwareMap.dcMotor.get(skystoneNames.rightCollectorMotor);
        modeSwitch = hardwareMap.servo.get(skystoneNames.modeSwitch);
        //leftDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.leftDistance);
        //frontDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.frontDistance);
        //rightDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.rightDistance);
        //backLeftDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backLeftDistance);
        //backRightDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backRightDistance);
        //Motor Settings

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRotation.setPosition(SKYSTONEConstants.straight);
        //frontBase.setPosition(SKYSTONEAutonomousConstants.fbStartPosition);
        //backBase.setPosition(SKYSTONEAutonomousConstants.bbStartPosition);
        // Front is really back claw
        //frontBase.setPosition(SKYSTONEAutonomousConstants.bbUp);
        //frontGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
        rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        //backBase.setPosition(SKYSTONEAutonomousConstants.fbUp);
        //backGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);

    }



    //Methods
    //Shortcuts


    //Drive Stuff
    //Preferably Do Not Touch
    void drive(double direction, double velocity, double rotationVelocity) {
        SKYSTONEClass.Wheels w = getWheels(direction, velocity, rotationVelocity);
        lf.setPower(w.lf);
        rf.setPower(w.rf);
        lb.setPower(w.lr);
        rb.setPower(w.rr);
        telemetry.addData("Powers", String.format(Locale.US, "lf %.2f lr %.2f rf %.2f rr %.2f", w.lf, w.lr, w.rf, w.rr));
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
    private SKYSTONEClass.Wheels getWheels(double direction, double velocity, double rotationVelocity) {
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

        return new SKYSTONEClass.Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }
    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }



    //Drivetrain
    void readEncoders(){
        telemetry.addData(
                "Encoders", "lf: " + lf.getCurrentPosition()
                        + " lb: " + lb.getCurrentPosition()
                        + " rf: " + rf.getCurrentPosition()
                        + " rb: "+ rb.getCurrentPosition()
                        + " elevator: " + elevator.getCurrentPosition()
                        + " slide motor: " + clawSlide.getCurrentPosition()
        );
    }


    //Servo Movement
    void grabStones (double closingDegrees) { clawServo.setPosition(closingDegrees); }
    void rotateStackingClaw(double turningDegrees) { clawRotation.setPosition(turningDegrees); }
    void runCollectorServos(double collectorPower){
        rightCollectorMotor.setPower(collectorPower);
        leftCollectorMotor.setPower(-collectorPower);
    }
    void setCapServo (double turningDegrees) { capServo.setPosition(turningDegrees); }
    void moveFrontClaw (double flClawPosition, double frClawPosition){
        //frontBase.setPosition(flClawPosition);
        //frontGrabber.setPosition(frClawPosition);
    }
    //Motor Movement
    void runElevatorMotors(double power){
        elevator.setPower(power);
    }
    void runWithEncoder(double power, int ticks, DcMotor...motors){
        ElapsedTime time = new ElapsedTime();
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        while(anyBusy(5, ticks, motors) && time.milliseconds()<2500){
            for(DcMotor motor : motors){
                Log.d("Motor " + motor.getPortNumber(), motor.getCurrentPosition() + "");
            }
        }
        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    void runWithEncoder(double power, int ticks, int tolerance, DcMotor...motors){
        ElapsedTime time = new ElapsedTime();
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        while(anyBusy(tolerance, ticks, motors) && time.milliseconds()<2500){
            for(DcMotor motor : motors){
                Log.d("Motor " + motor.getPortNumber(), motor.getCurrentPosition() + "");
            }
        }
        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    void runWithEncoderBegin(double power, int ticks, DcMotor...motors){
        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
    }

    void runWithEncoderEnd(int ticks, DcMotor...motors){
        ElapsedTime time = new ElapsedTime();
        while(anyBusy(5, ticks, motors) && time.milliseconds()<2500){
            for(DcMotor motor : motors){
                Log.d("Skystone Motor " + motor.getPortNumber(), motor.getCurrentPosition() + "");
            }
        }
        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    /*
    public int automatedStones(int stage, boolean isFast) {
        Moto mrow = (Servo) collectionRotationServo;
        if (stage == 0) {

        } else if(stage == 1) {
            mrow.setPosition(0.488);
        } else if(stage == 2) {

        } else if(stage == 3) {

        } else if(stage == 5) {

        }else if(stage == 6) {

        }
        return stage;
    }
    */
    boolean anyBusy(int tolerance, int targetPosition, DcMotor...motors){
        for(DcMotor motor : motors){
            if(Math.abs(targetPosition-motor.getCurrentPosition())>tolerance){
                return true;
            }
        }
        return false;
    }



    void resetAutonomous() {
        clawRotation.setPosition(SKYSTONEConstants.straight);
        clawServo.setPosition(SKYSTONEConstants.tighten);
        clawSlide.setPower(0.3);
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        clawSlide.setPower(0);
        //TODO: Replace with driveWithEncoder
        //leftElevator.setTargetPosition(SKYSTONEConstants.startingElevatorHeight);
        //rightElevator.setTargetPosition(SKYSTONEConstants.startingElevatorHeight);
    }


}
