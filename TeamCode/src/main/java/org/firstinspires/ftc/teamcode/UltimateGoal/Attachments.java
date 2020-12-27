package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.baseBot.Drivetrain;

public class Attachments extends Drivetrain {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    // private DcMotor flywheelMotor = null;
    private DcMotor leftFlywheelMotor = null;
    private DcMotor rightFlywheelMotor = null;
    // private Servo ringPusher = null;

    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        // flywheelMotor  = hardwareMap.get(DcMotor.class, "flywheel_motor");
        leftFlywheelMotor = hardwareMap.get(DcMotor.class, "flywheel_motor_left");
        rightFlywheelMotor = hardwareMap.get(DcMotor.class, "flywheel_motor_right");
        // ringPusher = hardwareMap.servo.get("ring_pusher");
        // flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFlywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFlywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        // flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //HardwareMaps
        initializeDriveTrain(hardwareMap, telemetry_);

    }

    void runFlywheelMotor(double power) {
        leftFlywheelMotor.setPower(-power);
        rightFlywheelMotor.setPower(power);
    }

    /* void runFlywheelMotor(double power) {
        flywheelMotor.setPower(power);
    }*/

    /* void setRingPusher(double position) {
        ringPusher.setPosition(position);
    } */
}
