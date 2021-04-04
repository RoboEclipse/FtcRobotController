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
public class blueLeftAutonomousDetectionPreLoad extends AutonomousMethods {
    private FtcDashboard dashboard;
    String detection;
    double dropDelay;

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
                .splineTo(new Vector2d(7, 25), Math.toRadians(-90))
                .splineTo(new Vector2d(-6, 12), Math.toRadians(-180))
                .splineTo(new Vector2d(-39, 24), Math.toRadians(120))
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
        detection = getWobbleDropPose();
        int wobbleDropx;
        int wobbleDropy;
        if (detection.equals("Quad")) {
            wobbleDropx = 44;
            wobbleDropy = 46;
            dropDelay = 2.8;
        } else if (detection.equals("Single")) {
            wobbleDropx = 22;
            wobbleDropy = 31;
            dropDelay = 2.7;
        } else {
            wobbleDropx = -2;
            wobbleDropy = 54;
            dropDelay = 1.8;
        }
        Pose2d wobbleDropPose = new Pose2d(wobbleDropx, wobbleDropy, Math.toRadians(0));

        //dashboard.stopCameraStream();
        Pose2d wobbleBackPose = wobbleDropPose.minus(new Pose2d(12, 0, Math.toRadians(0)));
        Pose2d wobbleDropPose2 = wobbleDropPose.minus(new Pose2d(0, 6, 0));
        Pose2d wobbleBackPose2 = wobbleDropPose2.minus(new Pose2d(12, 0, Math.toRadians(0)));

        //Trajectories are defined here so that wobbleDropx/y is actually correct
        Trajectory dropFirstWobble = drive.trajectoryBuilder(toRing.end())
                .splineToConstantHeading(new Vector2d(-36, 54), 0) //Goes left to avoid rings
                .addTemporalMarker(0.2, () -> { //TODO: Might need adjustments
                    hoverWobble();
                })
                .addTemporalMarker(1.2, () -> {
                    prepShooter();
                })
                .addTemporalMarker(dropDelay, () -> {
                    setWobbleClaw(false);
                })
                .addTemporalMarker(dropDelay + 0.2, () -> {
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
                .addTemporalMarker(dropDelay + 0.6, () -> {
                    lowerWobble();
                })
                .splineToSplineHeading(new Pose2d(-6, 26, Math.toRadians(0)), 0) // Goes to shooting position
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
                .splineTo(new Vector2d(-48, 42), Math.toRadians(90)) // Drives around rings
                .splineTo(new Vector2d(-36, 54), Math.toRadians(0))
                //TODO: Make proper vector
                .splineTo(new Vector2d(wobbleDropPose2.getX(), wobbleDropPose2.getY()), 0) //Drives back to correct spot for wobble drop off
                .build();
        Trajectory park = drive.trajectoryBuilder(dropSecondWobble.end())
                .splineToSplineHeading(wobbleBackPose2, 0)
                .splineTo(new Vector2d(12, 24), 0)
                .build();
        //Wobble drop should be at the end of the previous or at the beginning of the next one
//        Trajectory goShoot = drive.trajectoryBuilder(dropSecondWobble.end())
//
//                .build();

        drive.followTrajectory(dropFirstWobble);
        sleep(360);
        shootRings();
        sleep(720);

        drive.followTrajectory(toSecondWobble);
        setWobbleClaw(true);
        sleep(480);
        raiseWobble();
        //Servo drop wobble
        //drive.followTrajectory(pickupSecondWobble);
        //Servo grab wobble
        drive.followTrajectory(dropSecondWobble);
        hoverWobble();
        sleep(720);
        setWobbleClaw(false);
        sleep(300);
        raiseWobble();
        drive.followTrajectory(park);
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


