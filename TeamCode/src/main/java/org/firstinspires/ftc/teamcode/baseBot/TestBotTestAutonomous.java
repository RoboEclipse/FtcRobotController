package org.firstinspires.ftc.teamcode.baseBot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.UltimateGoal.AutonomousMethods;

@Autonomous(name="TestBotTestAutonomous", group="Linear Opmode")
public class TestBotTestAutonomous extends AutonomousMethods {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double speed = 1;

    @Override
    public void runOpMode() {
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        while (!isStarted()) {
            synchronized (this) {
                try {
                    //telemetry.addData("Distance", skystoneClass.getBackDistance() + "");
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            encoderStraightDrive(9, 0.6);
            sleep(1000);
            encoderStrafeDriveInchesRight(3, 0.36);
            sleep(1000);
            encoderStrafeDriveInchesRight(-3, 0.36);
            sleep(1000);
            encoderStraightDrive(-9, 0.6);
            break;
        }
    }
}
