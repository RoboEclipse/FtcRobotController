package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UltimateGoal.AutoTransitioner;
import org.firstinspires.ftc.teamcode.UltimateGoal.AutonomousMethods;
import org.firstinspires.ftc.teamcode.UltimateGoal.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
@Autonomous(group = "drive")
public class blueLeftAutonomous extends AutonomousMethods {
    private FtcDashboard dashboard;

    // String for holding detection
    String detection = "";

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        telemetry.addData("Initialization", "In Progress");
        telemetry.update();

        boolean isRed = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        setWobbleClaw(true);

        initVuforia();
        initTfod();

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(tfod, 0);

        if (tfod != null) { //Maybe move into Autonomous Methods
            tfod.activate();
        }

        //Create Vectors and Poses
        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Vector2d firstDropPositionClose = new Vector2d(5,51);
        Vector2d firstDropPositionMid = new Vector2d(27,27);
        Vector2d firstDropPositionFar = new Vector2d(52,48);
        Vector2d ringVector = new Vector2d(-50, 39);
        Vector2d shootVector = new Vector2d(-5, 38);
        Vector2d secondGrabPositionClose = new Vector2d(-36, 20);
        Vector2d secondGrabPositionMid = new Vector2d(-33.75, 17);
        Vector2d secondGrabPositionFar = new Vector2d(-37, 28);

        //Generate constant trajectories
        Trajectory toRing = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    raiseWobble();
                    setWobbleClaw(true);
                    setShooterAngle(Constants.setShooterAngle);
                })
                .splineToConstantHeading(ringVector, 0) //Goes right in front of the ring
                .build();
        Trajectory toShoot = drive.trajectoryBuilder(new Pose2d(ringVector, 0), 0)
                .addTemporalMarker(0, () -> {
                    hoverWobble();
                })
                .addTemporalMarker(0.1, () -> {
                    prepShooter();
                })
                .splineToConstantHeading(new Vector2d(-30, 60), 0)
                .splineToConstantHeading(new Vector2d(-12, 60), 0)
                .splineToConstantHeading(shootVector, 0) // Goes to shooting position
                .build();
        //Generate variable trajectory sets
        Trajectory[] closeTrajectories = generateRoute(drive, firstDropPositionClose, secondGrabPositionClose);
        Trajectory[] midTrajectories = generateRoute(drive, firstDropPositionMid, secondGrabPositionMid);
        Trajectory[] farTrajectories = generateRoute(drive, firstDropPositionFar, secondGrabPositionFar);
        Trajectory[] driveTrajectories;

        telemetry.addData("Initialization", "Finished");
        telemetry.update();

        waitForStart();

        //Drive to ring
        drive.followTrajectory(toRing);
        //getWobbleDropPose
        sleep(1200);
        detection = getWobbleDropPose();
        //Set trajectories based on ring detection
        if (detection.equals("Quad")) {
            driveTrajectories = farTrajectories;
        } else if (detection.equals("Single")) {
            driveTrajectories = midTrajectories;
        } else {
            driveTrajectories = closeTrajectories;
        }
        //Go to shoot location and power up shooter motor
        drive.followTrajectory(toShoot);
        //Correct imu
        //TODO: Replace with refreshPose
        encoderTurn(0,1,3);
        drive.setPoseEstimate(new Pose2d(shootVector, 0));
        //Shoot
        sleep(1008);
        shootRings();
        //Drive to first goal drop position
        drive.followTrajectory(driveTrajectories[0]);
        //Drop first goal
        setWobbleClaw(false);
        sleep(360);
        raiseWobble();
        sleep(500);
        //Drive to second goal pickup location
        //TODO: Split trajectory, add a refreshPose right before grabbing
        drive.followTrajectory(driveTrajectories[1]);
        //Pick up second goal
        sleep(500);
        setWobbleClaw(true);
        sleep(500);
        hoverWobble();
        //Drive to second goal drop position
        drive.followTrajectory(driveTrajectories[2]);
        //Drop second goal
        setWobbleClaw(false);
        sleep(200);
        raiseWobble();
        sleep(500);
        //Park
        drive.followTrajectory(driveTrajectories[3]);
        AutoTransitioner.transitionOnStop(this, "TeleOp");
    }

    private Trajectory[] generateRoute(SampleMecanumDrive drive, Vector2d firstDropPosition, Vector2d secondGrabPosition){
        Trajectory[] output = new Trajectory[4];
        Vector2d shootVector = new Vector2d(-6, 34);
        Vector2d secondDropPosition = firstDropPosition.plus(new Vector2d(-3,3));

        Vector2d parkPosition = new Vector2d(11.5, 22);

        Trajectory dropFirstWobble = drive.trajectoryBuilder(new Pose2d(shootVector, 0), 0) //Start at shoot position
                .strafeTo(firstDropPosition) //Go to firstDropPosition
                .build();
        Trajectory getSecondWobble = drive.trajectoryBuilder(new Pose2d(firstDropPosition,0), 0)
                .addTemporalMarker(1.6, () -> {
                    lowerWobble();
                })
                .back(10)
                .splineToConstantHeading(new Vector2d(0, 38), 0)
                //.splineToConstantHeading(new Vector2d(-6, 38), 0)
                .splineTo(new Vector2d(7, 25), Math.toRadians(-90))
                .splineTo(new Vector2d(-16, 12), Math.toRadians(-180))
                .splineTo(secondGrabPosition, Math.toRadians(120))
                .build();
        Trajectory dropSecondWobble = drive.trajectoryBuilder(getSecondWobble.end())
                .splineTo(new Vector2d(-48, 42), Math.toRadians(90))
                .splineTo(new Vector2d(-36, 54), Math.toRadians(0))
                .splineTo(secondDropPosition, 0)
                .build();
        Trajectory park = drive.trajectoryBuilder(new Pose2d(secondDropPosition, 0),0)
                .strafeTo(parkPosition)
                .build();

        output[0] = dropFirstWobble;
        output[1] = getSecondWobble;
        output[2] = dropSecondWobble;
        output[3] = park;
        return output;
    }
}
