package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.baseBot.Drivetrain;

public class Attachments extends Drivetrain {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public Configuration names = new Configuration();
    public DcMotor collectionMotor, wobbleGoalMotor, shooterMotor;
    public Servo wobbleGoalServo, ringPushServo, elevatorServo, tiltServo, shooterTiltServo, sideArmServo;

    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Motors
        collectionMotor = hardwareMap.dcMotor.get(names.collectionMotor);
        wobbleGoalMotor = hardwareMap.dcMotor.get(names.wobbleGoalMotor);
        shooterMotor = hardwareMap.dcMotor.get(names.shooterMotor);

        // Servos
        wobbleGoalServo = hardwareMap.servo.get(names.wobbleGoalServo);
        ringPushServo = hardwareMap.servo.get(names.ringPushServo);
        elevatorServo = hardwareMap.servo.get(names.elevatorServo);
        tiltServo = hardwareMap.servo.get(names.tiltServo);
        shooterTiltServo = hardwareMap.servo.get(names.shooterTiltServo);
        sideArmServo = hardwareMap.servo.get(names.sideArmServo);

        //Sensors
        clawDistance = hardwareMap.get(DistanceSensor.class, names.clawDistance);
        frontDistance = hardwareMap.get(DistanceSensor.class, names.frontDistance);
        leftDistance = hardwareMap.get(DistanceSensor.class, names.leftDistance);

        // Motor initalization
        collectionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //HardwareMaps
        initializeDriveTrain(hardwareMap, telemetry_);
    }

    void setWobbleClaw(double position) {
        wobbleGoalServo.setPosition(position);
    }
    double getWobbleMotorPosition() {return wobbleGoalMotor.getCurrentPosition(); }
    void runWobbleMotor(double power) {
        wobbleGoalMotor.setPower(power);
    }
    void runCollector(double power) {
        collectionMotor.setPower(power);
    }
    void setRingPusher(double position){ringPushServo.setPosition(position);}
    void setElevator(double position){elevatorServo.setPosition(position);}
    void setTilt(double position){tiltServo.setPosition(position);}
    void runShooter(double power) {
        shooterMotor.setPower(-power);
    }
    void setShooterAngle(double position) {shooterTiltServo.setPosition(position); 
    void setSideArmServo(double position) {
        sideArmServo.setPosition(position);
    }

    double getShooterAngle() {return(shooterTiltServo.getPosition()); }
}
