package org.firstinspires.ftc.teamcode.roadRunner.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Vector;

public class blueLeftAutonomousTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        int wobbleDropx = 0;
        int wobbleDropy = 0;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory pickupFirstWobble = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-48, 48), 0) //Goes forward
                .build();

        Trajectory toRing = drive.trajectoryBuilder(pickupFirstWobble.end())
                .splineToConstantHeading(new Vector2d(-48, 36), 0) //Goes right
                .splineToConstantHeading(new Vector2d(-42, 36), 0) //Goes forward to detect ring
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(pickupFirstWobble);
        //Servo grab wobble
        drive.followTrajectory(toRing);
        //Detection happens here
        wobbleDropx = 36;
        wobbleDropy = 48;

        //Trajectories are defined here so that wobbleDropx/y is actually correct
        Trajectory dropFirstWobble = drive.trajectoryBuilder(toRing.end())
                .splineToConstantHeading(new Vector2d(-36, 54), 0) //Goes left to avoid rings
                .splineToSplineHeading(new Pose2d(wobbleDropx, wobbleDropy, Math.toRadians(0)), 0) //Drives to correct spot for wobble drop off
                .build();

        Trajectory pickupSecondWobble = drive.trajectoryBuilder(dropFirstWobble.end())
                .splineTo(new Vector2d(-48, 24), 72) //Drive back to pick-up second wobble goal
                .build();

        Trajectory dropSecondWobble = drive.trajectoryBuilder(pickupSecondWobble.end()) //Maybe merge with dropFirst Wobble because we just need the same start location and it will work
                .splineToSplineHeading(new Pose2d(-36, 54, Math.toRadians(0)), 0) //Drives to the left
                .splineToSplineHeading(new Pose2d(wobbleDropx, wobbleDropy, Math.toRadians(0)), 0) //Drives back to correct spot for wobble drop off
                .build();

        Trajectory goShoot = drive.trajectoryBuilder(dropSecondWobble.end())
                .splineToSplineHeading(new Pose2d(12, 12, Math.toRadians(0)), 0)
                .build();

        Trajectory pickup = drive.trajectoryBuilder(goShoot.end())
                .splineTo(new Vector2d(-24, 36), 18)
                .build();

        Trajectory park = drive.trajectoryBuilder(pickup.end())
                .splineTo(new Vector2d(12, 36), 0)
                .build();

        drive.followTrajectory(dropFirstWobble);
        //Servo drop wobble
        drive.followTrajectory(pickupSecondWobble);
        //Servo grab wobble
        drive.followTrajectory(dropSecondWobble);
        //Servo drop wobble
        drive.followTrajectory(goShoot);
        //Shoot rings
        drive.followTrajectory(pickup);
        //Spin motors to collect
        drive.followTrajectory(park);
    }
}
