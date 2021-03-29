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
public class newRouteTest extends AutonomousMethods {
    private FtcDashboard dashboard;
    String detected = "";

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        boolean isRed = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        initVuforia();
        initTfod();

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(tfod, 0);

        if (tfod != null) { //Maybe move into Autonomous Methods
            tfod.activate();
        }

        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory toRing = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    raiseWobble();
                    setWobbleClaw(true);
                    setShooterAngle(Constants.setShooterAngle);
                })
                .splineToConstantHeading(new Vector2d(-48, 36), 0) //Goes right in front of the ring
                .build();

        Trajectory toShoot = drive.trajectoryBuilder(new Pose2d(-48, 36))
                .build();

        Trajectory toCloseDropFirstWobbleGoal = drive.trajectoryBuilder(new Pose2d(-48, 36))
                .build();

        Trajectory toDriveToSecondWobbleGoal = drive.trajectoryBuilder(new Pose2d(-48, 36))
                .build();

        Trajectory toSecondWobble = drive.trajectoryBuilder(new Pose2d(-6, 38, Math.toRadians(0))) //Maybe merge with dropFirst Wobble because we just need the same start location and it will work
                .splineTo(new Vector2d(7, 25), Math.toRadians(-90))
                .splineTo(new Vector2d(-6, 12), Math.toRadians(-180))
                .splineTo(new Vector2d(-38, 23), Math.toRadians(145))
                .build();

        //Start spinning collection motors in the next trajectory
//        Trajectory pickup = drive.trajectoryBuilder(new Pose2d(-12, 12, Math.toRadians(0)))
//                .addTemporalMarker(0, () -> {
//                    setCollectorPower(1);
//                })
//                .splineTo(new Vector2d(-24, 36), 18)
//                .addDisplacementMarker(() -> {
//                    setCollectorPower(0);
//                })
//                .build();

        waitForStart();

    }
}
