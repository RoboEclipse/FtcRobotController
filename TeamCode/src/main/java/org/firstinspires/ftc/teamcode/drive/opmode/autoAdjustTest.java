package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoal.AutoTransitioner;
import org.firstinspires.ftc.teamcode.UltimateGoal.AutonomousMethods;
import org.firstinspires.ftc.teamcode.UltimateGoal.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class autoAdjustTest extends AutonomousMethods {
    private FtcDashboard dashboard;

    // String for holding detection
    String detection = "";

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        telemetry.addData("Initialization", "In Progress");
        telemetry.update();
        telemetry.addData("Initialization", "Finished");
        telemetry.update();

        waitForStart();

        lowerWobble();
        setWobbleClaw(false);
        telemetry.addData("Current Step", "Ready to grab");
        telemetry.update();
        sleep(360);
        autoAdjust(8);
        telemetry.addData("Current Step", "Reached optimal distance");
        telemetry.update();
        sleep(720);
        setWobbleClaw(true);
        sleep(720);
        raiseWobble();
        telemetry.addData("Current Step", "Finished grabbing");
        telemetry.update();

        sleep(720);
    }
}
