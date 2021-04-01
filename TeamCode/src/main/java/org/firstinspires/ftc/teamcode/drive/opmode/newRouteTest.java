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

import java.util.Vector;

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
        Vector2d ringVector = new Vector2d(-48, 36);
        Vector2d shootVector = new Vector2d(-6, 28);
        Vector2d secondGrabPosition = new Vector2d(-39, 24);
        Vector2d parkPosition = new Vector2d(12, 24);

        drive.setPoseEstimate(startPose);

        Trajectory toRing = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    raiseWobble();
                    setWobbleClaw(true);
                    setShooterAngle(Constants.setShooterAngle);
                })
                .splineToConstantHeading(ringVector, 0) //Goes right in front of the ring
                .build();
        Trajectory toShoot = drive.trajectoryBuilder(new Pose2d(ringVector, 0), 0)
                .splineTo(shootVector, 0) // Goes to shooting position
                .build();
        Trajectory dropFirstWobbleClose;
        Trajectory getSecondWobbleClose;
        Trajectory dropSecondWobbleClose;
        Trajectory parkClose;
        Trajectory dropFirstWobbleMid;
        Trajectory getSecondWobbleMid;
        Trajectory dropSecondWobbleMid;
        Trajectory parkMid;
        Trajectory dropFirstWobbleFar;
        Trajectory getSecondWobbleFar;
        Trajectory dropSecondWobbleFar;
        Trajectory parkFar;

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

    private void generateCloseRoute(SampleMecanumDrive drive, Vector2d shootVector, Vector2d secondGrabPosition, Vector2d parkPosition){
        Vector2d firstDropPosition = new Vector2d(0,54);
        Vector2d secondDropPosition = firstDropPosition.plus(new Vector2d(6,-6));

        Trajectory dropFirstWobbleClose = drive.trajectoryBuilder(new Pose2d(shootVector, 0), 0) //Start at shoot position
                .strafeTo(firstDropPosition) //Go to firstDropPosition
                .build();
        Trajectory getSecondWobbleClose = drive.trajectoryBuilder(new Pose2d(firstDropPosition,0), 0)
                .strafeTo(shootVector)
                .splineTo(new Vector2d(7, 25), Math.toRadians(-90))
                .splineTo(new Vector2d(-16, 12), Math.toRadians(-180))
                .splineTo(secondGrabPosition, Math.toRadians(120))
                .build();
        Trajectory dropSecondWobbleClose = drive.trajectoryBuilder(new Pose2d(secondGrabPosition, 120), 120)
                .splineTo(secondDropPosition, 0)
                .build();
        Trajectory parkClose = drive.trajectoryBuilder(new Pose2d(secondDropPosition, 0),0)
                .strafeTo(parkPosition)
                .build();


    }
    public void generateMidRoute(SampleMecanumDrive drive, Vector2d shootVector, Vector2d secondGrabPosition, Vector2d parkPosition){
        Vector2d firstDropPosition = new Vector2d(0,54);
        Vector2d secondDropPosition = firstDropPosition.plus(new Vector2d(6,-6));

        Trajectory dropFirstWobbleClose = drive.trajectoryBuilder(new Pose2d(shootVector, 0), 0) //Start at shoot position
                .strafeTo(firstDropPosition) //Go to firstDropPosition
                .build();
        Trajectory getSecondWobbleClose = drive.trajectoryBuilder(new Pose2d(firstDropPosition,0), 0)
                .strafeTo(shootVector)
                .splineTo(new Vector2d(7, 25), Math.toRadians(-90))
                .splineTo(new Vector2d(-16, 12), Math.toRadians(-180))
                .splineTo(secondGrabPosition, Math.toRadians(120))
                .build();
        Trajectory dropSecondWobbleClose = drive.trajectoryBuilder(new Pose2d(secondGrabPosition, 120), 120)
                .splineTo(secondDropPosition, 0)
                .build();
        Trajectory parkClose = drive.trajectoryBuilder(new Pose2d(secondDropPosition, 0),0)
                .strafeTo(parkPosition)
                .build();
    }
    public void generateFarRoute(SampleMecanumDrive drive, Vector2d shootVector, Vector2d secondGrabPosition, Vector2d parkPosition){
        Vector2d firstDropPosition = new Vector2d(0,54);
        Vector2d secondDropPosition = firstDropPosition.plus(new Vector2d(6,-6));

        Trajectory dropFirstWobbleClose = drive.trajectoryBuilder(new Pose2d(shootVector, 0), 0) //Start at shoot position
                .strafeTo(firstDropPosition) //Go to firstDropPosition
                .build();
        Trajectory getSecondWobbleClose = drive.trajectoryBuilder(new Pose2d(firstDropPosition,0), 0)
                .strafeTo(shootVector)
                .splineTo(new Vector2d(7, 25), Math.toRadians(-90))
                .splineTo(new Vector2d(-16, 12), Math.toRadians(-180))
                .splineTo(secondGrabPosition, Math.toRadians(120))
                .build();
        Trajectory dropSecondWobbleClose = drive.trajectoryBuilder(new Pose2d(secondGrabPosition, 120), 120)
                .splineTo(secondDropPosition, 0)
                .build();
        Trajectory parkClose = drive.trajectoryBuilder(new Pose2d(secondDropPosition, 0),0)
                .strafeTo(parkPosition)
                .build();
    }
}
