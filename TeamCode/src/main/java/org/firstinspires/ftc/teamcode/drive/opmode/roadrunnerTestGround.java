package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoal.AutonomousMethods;
import org.firstinspires.ftc.teamcode.UltimateGoal.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class roadrunnerTestGround extends AutonomousMethods {
    private FtcDashboard dashboard;
    String detected = "";

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        boolean isRed = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        dashboard = FtcDashboard.getInstance();

        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory test = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-6, 38), 0)
                .splineTo(new Vector2d(7, 25), Math.toRadians(-90))
                .splineTo(new Vector2d(-6, 12), Math.toRadians(-180))
                .splineTo(new Vector2d(-38, 23), Math.toRadians(145))
                .splineTo(new Vector2d(-48, 42), Math.toRadians(90))
                .splineTo(new Vector2d(-36, 54), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;
        //Servo grab wobble
        drive.followTrajectory(test);
    }
}
