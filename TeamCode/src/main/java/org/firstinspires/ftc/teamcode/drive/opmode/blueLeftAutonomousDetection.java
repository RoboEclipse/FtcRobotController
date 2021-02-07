package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.UltimateGoal.AutonomousMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Vector;
@Config
@Autonomous(group = "drive")
public class blueLeftAutonomousDetection extends AutonomousMethods {
    @Override
    public void runOpMode() {
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
                .splineToConstantHeading(new Vector2d(-48, 36), 0) //Goes right
                .splineToConstantHeading(new Vector2d(-42, 36), 0) //Goes forward to detect ring
                .build();

        waitForStart();

        if(isStopRequested()) return;
        //Servo grab wobble
        sleep(1000);
        drive.followTrajectory(toRing);
        Pose2d wobbleDropPose = getWobbleDropPose(isRed);

        //Trajectories are defined here so that wobbleDropx/y is actually correct
        Trajectory dropFirstWobble = drive.trajectoryBuilder(toRing.end())
                .splineToConstantHeading(new Vector2d(-36, 54), 0) //Goes left to avoid rings
                .addTemporalMarker(1, () -> {
                    telemetry.addData("Test", "This should run 1 second after the robot goes to drop 1st wobble");
                    telemetry.update();
                })
                .splineToSplineHeading(wobbleDropPose, 0) //Drives to correct spot for wobble drop off
                .build();
        //Wobble drop should be at the end of the previous or at the beginning of the next one
        Trajectory pickupSecondWobble = drive.trajectoryBuilder(dropFirstWobble.end())
                .splineTo(new Vector2d(-48, 24), 120) //Drive back to pick-up second wobble goal
                .build();
        //Wobble pick up should be at the end of the previous or at the beginning of the next one
        Trajectory dropSecondWobble = drive.trajectoryBuilder(pickupSecondWobble.end()) //Maybe merge with dropFirst Wobble because we just need the same start location and it will work
                .splineToSplineHeading(new Pose2d(-36, 54, Math.toRadians(0)), 0) //Drives to the left
                .splineToSplineHeading(wobbleDropPose, 0) //Drives back to correct spot for wobble drop off
                .build();
        //Wobble drop should be at the end of the previous or at the beginning of the next one
        Trajectory goShoot = drive.trajectoryBuilder(dropSecondWobble.end())
                .splineToSplineHeading(new Pose2d(-9, 12, Math.toRadians(0)), 0)
                .build();
        //Start spinning collection motors in the next trajectory
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
