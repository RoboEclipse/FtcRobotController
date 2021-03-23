package org.firstinspires.ftc.teamcode.drive.opmode;

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
public class blueLeftAutonomousDetection extends AutonomousMethods {
    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        boolean isRed = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initVuforia();
        initTfod();

        if (tfod != null) { //Maybe move into Autonomous Methods
            tfod.activate();
        }

        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory toRing = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    grabWobble();
                    setShooterAngle(Constants.setShooterAngle);
                })
                .splineToConstantHeading(new Vector2d(-48, 36), 0) //Goes right
                .splineToConstantHeading(new Vector2d(-42, 36), 0) //Goes forward to detect ring
                .build();

        waitForStart();

        if(isStopRequested()) return;
        //Servo grab wobble
        drive.followTrajectory(toRing);
        Pose2d wobbleDropPose = getWobbleDropPose(isRed);
        Pose2d wobbleBackPose = wobbleDropPose.minus(new Pose2d(12, 0, -45));

        //Trajectories are defined here so that wobbleDropx/y is actually correct
        Trajectory dropFirstWobble = drive.trajectoryBuilder(toRing.end())
                .splineToConstantHeading(new Vector2d(-36, 54), 0) //Goes left to avoid rings
                .splineToSplineHeading(wobbleDropPose, 0) //Drives to correct spot for wobble drop off
                .addTemporalMarker(1.5, () -> { //TODO: Might need adjustments
                    dropWobble();
                })
                .splineToSplineHeading(wobbleBackPose, 0)
                .build();
        drive.followTrajectory(dropFirstWobble);
        //Wobble drop should be at the end of the previous or at the beginning of the next one
        Trajectory pickupSecondWobble = drive.trajectoryBuilder(dropFirstWobble.end())
                .addTemporalMarker(0, () -> {
                    lowerWobble();
                })
                .splineTo(new Vector2d(-24, 12), 0)
                .splineTo(new Vector2d(-48, 24), 120) //Drive back to pick-up second wobble goal
                .addDisplacementMarker(() -> {
                    grabWobble();
                })
                .build();
        //Wobble pick up should be at the end of the previous or at the beginning of the next one
        Trajectory dropSecondWobble = drive.trajectoryBuilder(pickupSecondWobble.end()) //Maybe merge with dropFirst Wobble because we just need the same start location and it will work
                .splineToSplineHeading(new Pose2d(-36, 54, Math.toRadians(0)), 0) //Drives to the left
                .splineToSplineHeading(wobbleDropPose, 0) //Drives back to correct spot for wobble drop off
                .addTemporalMarker(1.5, () -> { //TODO: Might need adjustments
                    dropWobble();
                })
                .splineToSplineHeading(wobbleBackPose, 0)
                .build();
        //Wobble drop should be at the end of the previous or at the beginning of the next one
        Trajectory goShoot = drive.trajectoryBuilder(dropSecondWobble.end())
                .splineToSplineHeading(new Pose2d(-12, 12, Math.toRadians(0)), 0)
                .build();
        //Start spinning collection motors in the next trajectory
        Trajectory pickup = drive.trajectoryBuilder(goShoot.end())
                .addTemporalMarker(0, () -> {
                    setCollectorPower(1);
                })
                .splineTo(new Vector2d(-24, 36), 18)
                .addDisplacementMarker(() -> {
                    setCollectorPower(0);
                })
                .build();

        Trajectory park = drive.trajectoryBuilder(pickup.end())
                .splineTo(new Vector2d(12, 36), 0)
                .build();

//        drive.followTrajectory(dropFirstWobble);
        //Servo drop wobble
        drive.followTrajectory(pickupSecondWobble);
        //Servo grab wobble
        drive.followTrajectory(dropSecondWobble);
        //Servo drop wobble
        drive.followTrajectory(goShoot);
        //Shoot rings
        shootRings(Constants.shooterPower);
        sleep(720);
        drive.followTrajectory(pickup);
        //Spin motors to collect
        drive.followTrajectory(park);
    }
}
