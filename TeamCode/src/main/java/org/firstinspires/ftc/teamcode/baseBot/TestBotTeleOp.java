package org.firstinspires.ftc.teamcode.baseBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TestBotTeleOp", group="Iterative Opmode")
public class TestBotTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain myRobot = new Drivetrain();

    public void init() {
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public void init_loop() {}

    public void start() {
        runtime.reset();
    }

    public void loop() {
        //Drive motor controls
        double lx = -gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double speedMultiplier = 1;
        double rotationMultiplier = .8;

        if(gamepad1.dpad_up){
            ly=-1;
            lx=0;
            speedMultiplier = 0.3;
        }
        else if(gamepad1.dpad_down){
            ly=1;
            lx=0;
            speedMultiplier = 0.3;
        }
        if(gamepad1.dpad_left){
            lx=1;
            ly=0;
            speedMultiplier = 0.6;
        }
        else if(gamepad1.dpad_right){
            lx=-1;
            ly=0;
            speedMultiplier = 0.6;
        }


        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);
    }

    public void stop() {}
}
