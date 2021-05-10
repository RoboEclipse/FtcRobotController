 /* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.UltimateGoal;

import android.util.Log;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


 @TeleOp(name="TeleOp", group="Iterative Opmode")
//@Disabled
public class UltimateGoalTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Attachments myRobot = new Attachments();
    private double prevTime = 0;
    private double prevPushTime = 0;
    private double wobbleServoPosition = Constants.wobbleClose;
    private double wobbleMotorPower = Constants.wobbleHoldingPower;
    private double collectorPower = 0;
    private double shooterPower = 0;
    private double ringPushPosition = Constants.ringPushBack;
    private double elevatorPosition = Constants.elevatorBottom;
    private double tiltPosition = Constants.bottomTilt;
    private double shooterAngle = Constants.setShooterAngle;
    private double rightSideArmPosition = Constants.rightSideArmIn;
    private boolean ringPushReturn = false;
    private int ringPushStep = -1;
    private boolean wobbleArmBack = false;
    private boolean bPressed = false;
    private double referenceAngle = 0;
    private boolean inAutoPowerShot = false;
    private boolean useWobblePower = true;
    private boolean turningToZero = false;
    private Trajectory powerShot;
    private Pose2d startPosition;
    private Pose2d currentPosition;
    SampleMecanumDrive drive;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        startPosition = new Pose2d(-63, 41, Math.toRadians(180));
        Vector2d shootPosition = new Vector2d(-24, 15);
        powerShot = drive.trajectoryBuilder(startPosition) //Start at shoot position
                .strafeTo(shootPosition) //Go to firstDropPosition
//                .splineToSplineHeading(new Pose2d(shootPosition, Math.toRadians(0)),Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    turningToZero = true;
                    useWobblePower = false;
                    setWobbleMotorPosition(0.9, Constants.wobbleBottomTeleOp);
                    wobbleArmBack = false;
                    wobbleServoPosition = Constants.wobbleOpen;
                })
                .build();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Drive motor controls
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier = 1;
        double rotationMultiplier = .8;
        if(gamepad1.dpad_up){
            ly=1;
            lx=0;
            speedMultiplier = 0.3;
        }
        else if(gamepad1.dpad_down){
            ly=-1;
            lx=0;
            speedMultiplier = 0.3;
        }
        if(gamepad1.dpad_left){
            lx=-1;
            ly=0;
            speedMultiplier = 0.6;
        }
        else if(gamepad1.dpad_right){
            lx=1;
            ly=0;
            speedMultiplier = 0.6;
        }
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;
        if(!inAutoPowerShot){
            myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);
        }


        double currentAngle = myRobot.getAngle();
        //TODO: Make this override properly
        //Automation for powershot
        if (gamepad1.left_bumper) {
            noPIDTurn(referenceAngle+2, Constants.imuTurnSpeed);
        } else if (gamepad1.right_bumper) {
            noPIDTurn(referenceAngle-2, Constants.imuTurnSpeed);
        }
        else{
            referenceAngle = Math.toDegrees(currentAngle);
        }

        //Collection
        if (gamepad2.left_trigger > 0.3) {
            if (Math.abs(elevatorPosition - Constants.elevatorTop) <= 0.05) {
                tiltPosition = Constants.bottomTilt;
                elevatorPosition = Constants.elevatorBottom;
                collectorPower = 0;
            } else {
                collectorPower = -Constants.collectionPower;
            }
        } else if (gamepad2.right_trigger > 0.3) {
            collectorPower = Constants.collectionPower;
        } else {
            collectorPower = 0;
        }

        //Ring pusher
        if (ringPushReturn && (runtime.milliseconds() - prevPushTime >= 180)) {
            ringPushPosition = Constants.ringPushBack;
            ringPushReturn = false;
        }
        if (gamepad2.left_bumper) {
            ringPushPosition = Constants.ringPush;
            ringPushReturn = true;
            prevPushTime = runtime.milliseconds();
        } else if (gamepad2.right_bumper) {
            ringPushStep = 1;
            rightSideArmPosition = Constants.rightSideArmOut;
//            rightSideArmPosition = Constants.rightSideArmOut;
//            ringPushPosition = Constants.ringPushBack;
        }
        if ((ringPushStep != -1) && (runtime.milliseconds() - prevTime >= 360)) {
            ringPushPosition = Constants.ringPush;
            ringPushReturn = true;
            prevPushTime = runtime.milliseconds();
            if (ringPushStep != 2) {
                ringPushStep++;
                prevTime = runtime.milliseconds();
            } else {
                ringPushStep = -1;
            }
        }


        //Elevator and tilt
        if (gamepad2.b) {
            tiltPosition = Constants.bottomTilt;
            elevatorPosition = Constants.elevatorBottom;
        } else if (gamepad2.a) {
            tiltPosition = Constants.topTilt;
            elevatorPosition = Constants.elevatorTop;
            shooterPower = Constants.shooterPower;
        }

        //Shooter
        if (gamepad2.dpad_up) {
            shooterPower = Constants.shooterPower;
        }
        /*
        else if (gamepad2.dpad_left) {
            shooterPower = 0.48;
        } else if (gamepad2.dpad_right) {
            shooterPower = 0.49;

        }
        */
        else if (gamepad2.dpad_down) {
            shooterPower = 0;
        }
        if (shooterPower < 0) {
            shooterPower = 0;
        } else if (shooterPower > 1) {
            shooterPower = 1;
        }



        if (gamepad2.y){
            shooterAngle = Constants.setShooterAngle;
        }

        if (gamepad2.x) {
            tiltPosition = Constants.topTilt;
            elevatorPosition = Constants.elevatorTop;
            shooterPower = Constants.shooterPower;
            imuTurn(0, 0.5);
        }

        //Shooter Angle
        double shooterJoystick = -gamepad2.left_stick_y;
        shooterAngle += shooterJoystick*0.005;
        if(shooterAngle>1){
            shooterAngle = 1;
        }
        if(shooterAngle<0){
            shooterAngle = 0;
        }

        double wobbleJoystick = -gamepad2.right_stick_y;
        if (Math.abs(wobbleJoystick) > 0.1) {
            useWobblePower = true;
            wobbleArmBack = false;
            myRobot.wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleMotorPower = wobbleJoystick * 0.75;
        } else {
            wobbleMotorPower = 0;
        }

        if (wobbleArmBack && !myRobot.wobbleGoalMotor.isBusy()) {
            useWobblePower = true;
            wobbleMotorPower = 0;
            wobbleArmBack = false;
        }

        //Wobble motor
        if (gamepad1.left_trigger > 0.3) {
//            wobbleMotorPower = Constants.wobbleLowerPower;
            useWobblePower = false;
            setWobbleMotorPosition(0.9, 0);
            wobbleArmBack = true;
        } else if (gamepad1.right_trigger > 0.3) {
//            wobbleMotorPower = Constants.wobbleRaisePower;
            useWobblePower = false;
            setWobbleMotorPosition(0.9, Constants.wobbleBottomTeleOp);
            wobbleArmBack = false;
//            useWobblePower = true;
        } else {
//            wobbleMotorPower = Constants.wobbleHoldingPower;
        }

        //Wobble claw
        if (gamepad1.x){
            wobbleServoPosition = Constants.wobbleOpen;
        } else if (gamepad1.y){
            wobbleServoPosition = Constants.wobbleClose;
        }

        /*
        if (shooterJoystick > 0.36) {
            shooterAngle += Constants.shooterAngleIncrease;
        } else if (shooterJoystick < -0.36) {
            shooterAngle -= Constants.shooterAngleIncrease;
        } else if (gamepad2.y) {
            shooterAngle = Constants.setShooterAngle;
        } else if (gamepad2.x) {
            // Automation here
        }
        */

        if(gamepad1.b && !bPressed){
            bPressed = true;
            if(rightSideArmPosition == Constants.rightSideArmOut){
                rightSideArmPosition = Constants.rightSideArmStraight;
            } else if (rightSideArmPosition == Constants.rightSideArmStraight) {
                rightSideArmPosition = Constants.rightSideArmIntake;
            } else {
                rightSideArmPosition = Constants.rightSideArmOut;
            }
        }
        else if(!gamepad1.b){
            bPressed = false;
        }
        if(gamepad1.a){
            rightSideArmPosition = Constants.rightSideArmIn;
        }

        //Autodrive
//        if(gamepad2.dpad_left){
//            imuTurn(180, 0.5);
//            double rightDistance = myRobot.getRightDistance();
//            if (rightDistance < 50) {
//                currentPosition = new Pose2d(-72 + 8.5, 63.875 - rightDistance, Math.toRadians(180));
//            } else {
//                currentPosition = startPosition;
//            }
//
//            drive.setPoseEstimate(currentPosition);
//            drive.followTrajectoryAsync(powerShot);
//            inAutoPowerShot = true;
//        }
//        if(inAutoPowerShot){
//            if(turningToZero){
//                if(imuTurn(0, 0.5)){
//                    inAutoPowerShot = false;
//                    turningToZero = false;
//
//                    myRobot.lf.setPower(0);
//                    myRobot.lb.setPower(0);
//                    myRobot.rf.setPower(0);
//                    myRobot.rb.setPower(0);
//
//                    tiltPosition = Constants.topTilt;
//                    elevatorPosition = Constants.elevatorTop;
//                    shooterPower = Constants.shooterPower;
//                }
//            } else {
//                drive.update();
//            }
//        }
//
//
//        if(gamepad2.dpad_right){
//            inAutoPowerShot = false;
//            turningToZero = false;
//        }

        if (useWobblePower) {
            myRobot.runWobbleMotor(wobbleMotorPower);
        }
        myRobot.setWobbleClaw(wobbleServoPosition);
        myRobot.runCollector(collectorPower);
        myRobot.setRingPusher(ringPushPosition);
        myRobot.setTilt(tiltPosition);
        myRobot.setElevator(elevatorPosition);
        myRobot.runShooter(shooterPower);
        myRobot.setShooterAngle(shooterAngle);
        myRobot.setSideArmServo(rightSideArmPosition);

        /*Should look like:
        2020-11-08 21:08:37.960 2298-2424/com.qualcomm.ftcrobotcontroller D/Encoders: Front: 3681 Left: -324 Right: -406
        2020-11-08 21:08:37.964 2298-2424/com.qualcomm.ftcrobotcontroller D/Encoders: Front: 3681 Left: -324 Right: -406
        */

        /*
        telemetry.addData("FrontDistance", myRobot.getFrontDistance());
        telemetry.addData("LeftDistance", myRobot.getLeftDistance());
        telemetry.addData("RightDistance", myRobot.getRightDistance());
        */
        telemetry.addData("Angle", Math.toDegrees(currentAngle));
        telemetry.addData("wobbleMotorPower", wobbleMotorPower);
        telemetry.addData("wobbleMotorPosition", myRobot.getWobbleMotorPosition());
        telemetry.addData("wobbleServoPosition", wobbleServoPosition);
        telemetry.addData("collectorPower", collectorPower);
        telemetry.addData("ringPushPosition", ringPushPosition);
        telemetry.addData("elevatorPosition", elevatorPosition);
        telemetry.addData("tiltPosition", tiltPosition);
        telemetry.addData("shooterPower", shooterPower);
        telemetry.addData("shooterAngle", shooterAngle);
        telemetry.addData("DriveEncoders",
                myRobot.getOdometryWheels()
        );
        telemetry.addData("ArmEncoder", myRobot.getWobbleMotorPosition());

        Log.d("wobbleMotorPower", String.valueOf(wobbleMotorPower));
        Log.d("wobbleServoPosition", String.valueOf(wobbleServoPosition));
        Log.d("collectorPower", String.valueOf(collectorPower));
        Log.d("ringPushPosition", String.valueOf(ringPushPosition));
        Log.d("elevatorPosition", String.valueOf(elevatorPosition));
        Log.d("tiltPosition", String.valueOf(tiltPosition));
        Log.d("shooterPower", String.valueOf(shooterPower));
        Log.d("shooterAngle", String.valueOf(shooterAngle));
        Log.d("Encoders",
            myRobot.getOdometryWheels()
        );
        Log.d("ArmEncoder", String.valueOf(myRobot.getWobbleMotorPosition()));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public void setWobbleMotorPosition(double speed, int position){
        myRobot.wobbleGoalMotor.setTargetPosition(position);
        myRobot.wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myRobot.wobbleGoalMotor.setPower(speed);
    }

    public boolean imuTurn(double targetAngle, double speed){
        //returns whether it's done
        double tolerance = 1;
        double maxError = 30;
        double minPower = 0.2;
        double currentAngle = Math.toDegrees(myRobot.getAngle());
        double error = currentAngle-targetAngle;
        if(Math.abs(error)<tolerance){
            return true;
        }
        double leftPower = (error/maxError)*speed;
        double rightPower = (error/maxError)*-speed;
        if(Math.abs(leftPower)<minPower){
            double sign = Math.signum(leftPower);
            leftPower = minPower* sign;
        }
        if(Math.abs(rightPower)<minPower){
            double sign = Math.signum(rightPower);
            rightPower = minPower * sign;
        }
        myRobot.lf.setPower(leftPower);
        myRobot.lb.setPower(leftPower);
        myRobot.rf.setPower(rightPower);
        myRobot.rb.setPower(rightPower);
        return false;
    }
    public void noPIDTurn(double targetAngle, double speed){
        double currentAngle = Math.toDegrees(myRobot.getAngle());
        double error = currentAngle-targetAngle;
        double leftPower = error*speed;
        double rightPower = error*-speed;
        myRobot.lf.setPower(leftPower);
        myRobot.lb.setPower(leftPower);
        myRobot.rf.setPower(rightPower);
        myRobot.rb.setPower(rightPower);
        /*
        while(error>tolerance){
            currentAngle = myRobot.getAngle();
            error = currentAngle-targetAngle;
        }
        */
    }
}

