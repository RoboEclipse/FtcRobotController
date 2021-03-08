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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp", group="Iterative Opmode")
//@Disabled
public class UltimateGoalTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Attachments myRobot = new Attachments();
    private double wobbleServoPosition = Constants.wobbleClose;
    private double wobbleMotorPower = Constants.wobbleHoldingPower;
    private double collectorPower = 0;
    private double shooterPower = 0;
    private double ringPushPosition = Constants.ringPushBack;
    private double elevatorPosition = Constants.elevatorBottom;
    private double tiltPosition = Constants.bottomTilt;
    private double shooterAngle = Constants.setShooterAngle;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
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
        double lx = -gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double speedMultiplier = 1;
        double rotationMultiplier = .8;
        if(gamepad1.dpad_up){
            ly=-1;
            lx=0;
            speedMultiplier = 0.3;
        }
        else if(gamepad1.dpad_down){
            ly=1;
            lx=0;
            speedMultiplier = 0.3;
        }
        if(gamepad1.dpad_left){
            lx=1;
            ly=0;
            speedMultiplier = 0.6;
        }
        else if(gamepad1.dpad_right){
            lx=-1;
            ly=0;
            speedMultiplier = 0.6;
        }
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;
        myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);



        //Wobble motor
        if (gamepad1.left_trigger > 0.3) {
            wobbleMotorPower = Constants.wobbleLowerPower;
        } else if (gamepad1.right_trigger > 0.3) {
            wobbleMotorPower = Constants.wobbleRaisePower;
        } else {
            wobbleMotorPower = Constants.wobbleHoldingPower;
        }

        //Wobble claw
        if (gamepad1.a){
            wobbleServoPosition = Constants.wobbleOpen;
        } else if (gamepad1.b){
            wobbleServoPosition = Constants.wobbleClose;
        }

        //Collection
        if (gamepad2.left_trigger > 0.3) {
            collectorPower = Constants.collectionPower;
        } else if (gamepad2.right_trigger > 0.3) {
            collectorPower = -Constants.collectionPower;
        } else {
            collectorPower = 0;
        }

        //Ring pusher
        if (gamepad2.left_bumper) {
            ringPushPosition = Constants.ringPush;
        } else if (gamepad2.left_bumper) {
            ringPushPosition = Constants.ringPushBack;
        }

        //Elevator and tilt
        if (gamepad2.b) {
            elevatorPosition = Constants.elevatorBottom;
            tiltPosition = Constants.bottomTilt;
        } else if (gamepad2.a) {
            elevatorPosition = Constants.elevatorTop;
            tiltPosition = Constants.topTilt;
        }

        //Shooter
        if (gamepad2.dpad_up) {
            shooterPower = Constants.shooterPower;
        } else {
            shooterPower = 0;
        }

        //Shooter Angle
        double shooterJoystick = gamepad2.left_stick_y;
        if (shooterJoystick > 0.36) {
            shooterAngle += Constants.shooterAngleIncrease;
        } else if (shooterJoystick < -0.36) {
            shooterAngle -= Constants.shooterAngleIncrease;
        } else if (gamepad2.y) {
            shooterAngle = Constants.setShooterAngle;
        } else if (gamepad2.x) {
            // Automation here
        }


        myRobot.runWobbleMotor(wobbleMotorPower);
        myRobot.setWobbleClaw(wobbleServoPosition);
        myRobot.runCollector(collectorPower);
        myRobot.setRingPusher(ringPushPosition);
        myRobot.setElevator(elevatorPosition);
        myRobot.setTilt(tiltPosition);
        myRobot.runShooter(shooterPower);
        myRobot.setShooterAngle(shooterAngle);

        /*Should look like:
        2020-11-08 21:08:37.960 2298-2424/com.qualcomm.ftcrobotcontroller D/Encoders: Front: 3681 Left: -324 Right: -406
        2020-11-08 21:08:37.964 2298-2424/com.qualcomm.ftcrobotcontroller D/Encoders: Front: 3681 Left: -324 Right: -406
        */

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


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}

