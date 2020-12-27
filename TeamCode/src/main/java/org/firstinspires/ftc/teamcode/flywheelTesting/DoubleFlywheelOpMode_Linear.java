package org.firstinspires.ftc.teamcode.flywheelTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DoubleFlywheelOpMode_Linear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    // change type of motor? need to double check
    private DcMotor leftFlywheelMotor = null;
    private DcMotor rightFlywheelMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // change names for deviceName based on name of motor
        leftFlywheelMotor = hardwareMap.get(DcMotor.class, "flywheel_motor_left");
        rightFlywheelMotor = hardwareMap.get(DcMotor.class, "flywheel_motor_right");

        // one motor is moving forward, other is moving in reverse, which one is forward and which one is reverse?
        // make sure to change accordingly
        leftFlywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFlywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // starts off without any power
        double flywheelPower = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Set flywheel power based on press
            if (gamepad1.a){
                // make sure to change the values accordingly after tests and make sure direction is correct
                flywheelPower = 1;
            }
            if (gamepad1.b) {
                flywheelPower = 0;
            }

            // Send calculated power to wheels
            leftFlywheelMotor.setPower(-flywheelPower);
            rightFlywheelMotor.setPower(flywheelPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "flywheel power: (%.2f)", flywheelPower);
            telemetry.update();
        }
    }
}
