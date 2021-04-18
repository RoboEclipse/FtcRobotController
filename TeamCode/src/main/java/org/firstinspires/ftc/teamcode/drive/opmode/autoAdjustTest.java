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

        // lowerWobble();
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

    private Trajectory[] generateRoute(SampleMecanumDrive drive, Vector2d firstDropPosition, Vector2d secondGrabPosition){
        Trajectory[] output = new Trajectory[6];
        Vector2d shootVector = new Vector2d(-6, 34);
        Vector2d prepVector = new Vector2d(0, 64);
        Vector2d cornerVector = new Vector2d(-64,64);
        Vector2d secondDropPosition = firstDropPosition.plus(new Vector2d(-3,3));

        Vector2d parkPosition = new Vector2d(11.5, 22);

        Trajectory dropFirstWobble = drive.trajectoryBuilder(new Pose2d(shootVector, 0), 0) //Start at shoot position
                .strafeTo(firstDropPosition) //Go to firstDropPosition
                .build();
        Trajectory toCornerFirst = drive.trajectoryBuilder(dropFirstWobble.end())
                .splineTo(prepVector, 0)
                .splineTo(cornerVector, 0)
                .build();
        Trajectory grabSecondWobble = drive.trajectoryBuilder(new Pose2d(toCornerFirst.end().vec(), Math.toRadians(-90)))
                .splineTo(secondGrabPosition, Math.toRadians(-90))
                .build();
        Trajectory toCornerSecond = drive.trajectoryBuilder(grabSecondWobble.end())
                .splineTo((cornerVector), Math.toRadians(-90))
                .build();
        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(toCornerSecond.end().vec(), 0))
                .splineTo(secondDropPosition, 0)
                .build();
        Trajectory park = drive.trajectoryBuilder(new Pose2d(secondDropPosition, 0),0)
                .strafeTo(parkPosition)
                .build();
        output[0] = dropFirstWobble;
        output[1] = toCornerFirst;
        output[2] = grabSecondWobble;
        output[3] = toCornerSecond;
        output[4] = dropSecondWobble;
        output[5] = park;
        return output;
    }

}
