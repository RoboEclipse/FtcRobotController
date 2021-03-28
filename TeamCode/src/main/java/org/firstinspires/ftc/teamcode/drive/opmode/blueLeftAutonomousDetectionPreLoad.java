package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.UltimateGoal.AutonomousMethods;
import org.firstinspires.ftc.teamcode.UltimateGoal.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Vector;
@Config
@Autonomous(group = "drive")
public class blueLeftAutonomousDetectionPreLoad extends AutonomousMethods {
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

        Trajectory toSecondWobble = drive.trajectoryBuilder(new Pose2d(-6, 38, Math.toRadians(0))) //Maybe merge with dropFirst Wobble because we just need the same start location and it will work
                .splineToSplineHeading(new Pose2d(12, 38, Math.toRadians(0)), 0)
                .splineTo(new Vector2d(-48, 12), 120)
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

        if(isStopRequested()) return;
        //Servo grab wobble
        drive.followTrajectory(toRing);
        sleep(500);
        Pose2d wobbleDropPose = getWobbleDropPose(isRed);
        //dashboard.stopCameraStream();
        Pose2d wobbleBackPose = wobbleDropPose.minus(new Pose2d(12, 0, Math.toRadians(0)));
        Pose2d wobbleDropPose2 = wobbleDropPose.minus(new Pose2d(6, -6, 0));
        Pose2d wobbleBackPose2 = wobbleDropPose2.minus(new Pose2d(12, 0, Math.toRadians(0)));

        //Trajectories are defined here so that wobbleDropx/y is actually correct
        Trajectory dropFirstWobble = drive.trajectoryBuilder(toRing.end())
                .splineToConstantHeading(new Vector2d(-36, 54), 0) //Goes left to avoid rings
                .addTemporalMarker(0.2, () -> { //TODO: Might need adjustments
                    hoverWobble();
                })
                .addTemporalMarker(1.8, () -> {
                    setWobbleClaw(false);
                })
                .addTemporalMarker(2, () -> {
                    raiseWobble();
                })
                .splineToSplineHeading(wobbleDropPose, 0) //Drives to correct spot for wobble drop off
//                .addTemporalMarker(1.2, () -> {
//                    setWobbleMotorPower(0.75);
//                })
//                .addTemporalMarker(2.2, () -> {
//                    setWobbleMotorPower(0);
//                })
                .splineToSplineHeading(wobbleBackPose, 0)
                .addTemporalMarker(2.4, () -> {
                    lowerWobble();
                })
                .splineToSplineHeading(new Pose2d(-6, 38, Math.toRadians(0)), 0)
                .build();

        //Wobble drop should be at the end of the previous or at the beginning of the next one
//        Trajectory pickupSecondWobble = drive.trajectoryBuilder(dropFirstWobble.end())
//                .addTemporalMarker(0, () -> {
//                    //lowerWobble();
//                })
//                .splineTo(new Vector2d(-24, 12), 0)
//                 //Drive back to pick-up second wobble goal
//                .addDisplacementMarker(() -> {
//                    //grabWobble();
//                })
//                .build();
        //Wobble pick up should be at the end of the previous or at the beginning of the next one
        Trajectory dropSecondWobble = drive.trajectoryBuilder(toSecondWobble.end()) //Maybe merge with dropFirst Wobble because we just need the same start location and it will work
                .splineToSplineHeading(new Pose2d(-36, 54, Math.toRadians(0)), 0) //Drives to the left
                .splineToSplineHeading(wobbleDropPose2, 0) //Drives back to correct spot for wobble drop off
                .addTemporalMarker(1.5, () -> { //TODO: Might need adjustments
                    //dropWobble();
                })
                .splineToSplineHeading(wobbleBackPose2, 0)
                .splineTo(new Vector2d(12, 36), 0)
                .build();
        //Wobble drop should be at the end of the previous or at the beginning of the next one
//        Trajectory goShoot = drive.trajectoryBuilder(dropSecondWobble.end())
//
//                .build();

        drive.followTrajectory(dropFirstWobble);
        sleep(360);
        shootRings(Constants.shooterPower);
        sleep(720);

        drive.followTrajectory(toSecondWobble);
        setWobbleClaw(true);
        sleep(240);
        raiseWobble();
        //Servo drop wobble
        //drive.followTrajectory(pickupSecondWobble);
        //Servo grab wobble
        drive.followTrajectory(dropSecondWobble);
        if (tfod != null) {
            tfod.shutdown();
        }
        //Servo drop wobble
        //drive.followTrajectory(goShoot);
        //Shoot rings

        //drive.followTrajectory(pickup);
        //Spin motors to collect
    }
}
