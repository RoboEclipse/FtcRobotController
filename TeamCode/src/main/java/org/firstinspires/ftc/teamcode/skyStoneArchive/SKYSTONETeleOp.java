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

package org.firstinspires.ftc.teamcode.skyStoneArchive;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="SKYSTONETeleOp", group="Iterative Opmode")
//@Disabled
public class SKYSTONETeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private SKYSTONEClass myRobot = new SKYSTONEClass();
    private double clawRotator = SKYSTONEConstants.straight;
    private double clawPosition = SKYSTONEConstants.loosen;
    private double leftFoundationPosition = SKYSTONEConstants.lDown;
    private double rightFoundationPosition = SKYSTONEConstants.rDown;
    private double collectorPower = 0;
    private double modeSwitchPosition = SKYSTONEConstants.stackingMode;
    private double capStonePosition = 0.33;
    private double flClawPosition = SKYSTONEConstants.frontHigh;
    private double frClawPosition = SKYSTONEConstants.frontLoosen;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        myRobot.frontColor.enableLed(false);
        myRobot.backColor.enableLed(false);
        myRobot.frontColor.red();
        myRobot.backColor.red();
        myRobot.capServo.setPosition(capStonePosition);
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

        myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);

        //Elevator controls
        double elevatorPower = -gamepad2.left_stick_y;
        boolean limitSwitch = myRobot.limitSwitch.getState();
        int elevatorPosition = myRobot.elevator.getCurrentPosition();
        if(!limitSwitch){
            myRobot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            myRobot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("ElevatorAction:","ResetingEncoder");
        }
        if(Math.abs(elevatorPower)<0.01 && limitSwitch){
            elevatorPower = 0.0005;
            telemetry.addData("ElevatorAction:","HoldingEncoder");
        }
        if(elevatorPower<0){
            if(!limitSwitch){
                elevatorPower = 0;
                telemetry.addData("ElevatorAction:","StoppingReduction");
            }
            if(elevatorPosition<700){
                elevatorPower *= 0.5;
                telemetry.addData("ElevatorAction:","SlowingDown");
            }
        }
        myRobot.runElevatorMotors(elevatorPower);

        //Slide controls
        double slidePower = gamepad2.right_stick_y;
        myRobot.clawSlide.setPower(slidePower);

        //Capstone Controls
        if(gamepad1.x) {
            capStonePosition = SKYSTONEConstants.cDown;
        } else if (gamepad1.y) {
            capStonePosition = SKYSTONEConstants.cUp;
        }
        myRobot.capServo.setPosition(capStonePosition);

        //Claw rotation
        int horizSlidePosition = myRobot.clawSlide.getCurrentPosition();
        if(gamepad2.dpad_left){
            clawRotator = SKYSTONEConstants.right90;
        }
        else if(gamepad2.dpad_right) {
            clawRotator = SKYSTONEConstants.left90;
        }
        else if(gamepad2.dpad_down){
            clawRotator = SKYSTONEConstants.oppositeSide;
        }
        else if(gamepad2.dpad_up){
            clawRotator = SKYSTONEConstants.straight;
        }

        //Claw controls
        if(gamepad2.x) {
            clawPosition = SKYSTONEConstants.tighten;
        }
        else if(gamepad2.y) {
            clawPosition = SKYSTONEConstants.loosen;
        }

        //Collector Servos
        if(gamepad2.right_trigger>0.3){
            collectorPower = -1;
        }
        else if(gamepad2.left_trigger>0.3){

                collectorPower = 1;
        }
        else{
            collectorPower = 0;
        }

        //Side Claw test{
        if(gamepad2.right_bumper){
            myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
            myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
            myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbStartPosition);
            myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbStartPosition);
        }
        if(gamepad2.left_bumper){
            myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbReady);
            myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsReady);
            myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbReady);
            myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsReady);
        }

        // TODO skystoneClass.moveFrontClaw(flClawPosition, frClawPosition);

        //Foundation Servo Control (testing)
        if(gamepad1.left_trigger>0.7){
            leftFoundationPosition = SKYSTONEConstants.lUp;
            rightFoundationPosition = SKYSTONEConstants.rUp;
        }
        if(gamepad1.right_trigger>0.7){
            leftFoundationPosition = SKYSTONEConstants.lDown;
            rightFoundationPosition = SKYSTONEConstants.rDown;

        }
        if(gamepad1.right_bumper){
            modeSwitchPosition = SKYSTONEConstants.deliverMode;
        }
        if(gamepad1.left_bumper){
            modeSwitchPosition = SKYSTONEConstants.stackingMode;
        }

        //Reset to Autonomous starting position
        /* if(gamepad1.x){
            skystoneClass.resetAutonomous();
            clawRotator = SKYSTONEConstants.straight;
        }*/


        //Autonomous Tests
        /*if(gamepad2.a){
            pickUpStone();
        } */
        myRobot.rotateStackingClaw(clawRotator);
        myRobot.grabStones(clawPosition);
        myRobot.runCollectorServos(collectorPower);
        myRobot.modeSwitch.setPosition(modeSwitchPosition);
        myRobot.leftFoundationServo.setPosition(leftFoundationPosition);
        myRobot.rightFoundationServo.setPosition(rightFoundationPosition);
        // Show the elapsed game time and wheel power.
        telemetry.addData("LimitSwitch", limitSwitch);
        telemetry.addData("ElevatorPower", elevatorPower);
        telemetry.addData("SlidePower", slidePower);
        telemetry.addData("ClawRotationPosition", clawRotator);
        telemetry.addData("ClawPosition", clawPosition);
        telemetry.addData("leftFoundationPosition", leftFoundationPosition);
        telemetry.addData("CollectorPower", collectorPower);
        telemetry.addData("CapstonePosition", capStonePosition);
        //telemetry.addData("ElevatorDistance", skystoneClass.getElevatorDistance());
        //telemetry.addData("BackDistance", skystoneClass.getBackDistance());
        myRobot.readEncoders();

        //Log
        Log.d("ElevatorPower", String.valueOf(elevatorPower));
        Log.d("SlidePower", String.valueOf(slidePower));
        Log.d("ClawRotationPosition", String.valueOf(clawRotator));
        Log.d("ClawPosition", String.valueOf(clawPosition));
        Log.d("LeftFoundationPosition", String.valueOf(leftFoundationPosition));

        Log.d("CollectorPower", String.valueOf(collectorPower));
        Log.d("Encoders",
        "lf: " + myRobot.lf.getCurrentPosition()
            + " lb: " + myRobot.lb.getCurrentPosition()
            + " rf: " + myRobot.rf.getCurrentPosition()
            + " rb: "+ myRobot.rb.getCurrentPosition()
            + " elevator: " + elevatorPosition
            + " slide motor: " + horizSlidePosition
        );

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
    private int stage = 0;
    private ElapsedTime timer = new ElapsedTime();
    /* private void pickUpStone(){
        //State 1: Raise elevator, then rotate claw
        if(stage == 0 && timer.milliseconds()>=2000){
            skystoneClass.runWithEncoder(0.5, 400, skystoneClass.elevator);
            clawRotator = SKYSTONEConstants.left90;
            timer.reset();
            stage++;
        }
        //State 2: Lower elevator, then grab stone
        if(stage == 1 && timer.milliseconds()>=2000){
            skystoneClass.runWithEncoder(0.5, -400, skystoneClass.elevator);
            clawPosition = SKYSTONEConstants.tighten;
            timer.reset();
            stage++;
        }
        //State 3: Straighten Claw
        if(stage == 2 && timer.milliseconds()>=2000){
            clawRotator = SKYSTONEConstants.straight;
            timer.reset();
            stage=0;
        }
    }*/

    void autoFoundationPull(){

    }
}

