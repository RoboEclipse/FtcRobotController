package org.firstinspires.ftc.teamcode.skyStoneArchive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.RevBulkData;

abstract class AutonomousMethods extends LinearOpMode {
    SKYSTONEDrivetrainClass myRobot;
    Localizer localizer;
    LocalizerReader localizerReader;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    //Hardware
    // The IMU sensor object
    //BNO055IMU imu;
    private Orientation angles;
    private int turnPValue = 15;

    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }



    void initializeDrivetrainAutonomous(HardwareMap hardwareMap, Telemetry telemetry, SKYSTONEDrivetrainClass inputRobot){
        myRobot = inputRobot;
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
        //Create localizer and localizerReader
        localizer = new Localizer(this.myRobot);
        localizerReader = LocalizerReader.INSTANCE;
        //Bind localizerReader to localizer
        localizerReader.setLocalizer(localizer);
        //Bind localizer to autoTransition
        LocalizerReader.transitionOnStop(this, "");
        this.telemetry = telemetry;

    }

    //Methods
    //TODO: Incorporate freeEncoderDrive, encoderDrive and runUsingEncoder
    void freeEncoderDrive(int lf, int rf, int lb, int rb, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElapsedTime time = new ElapsedTime();
        myRobot.lf.setTargetPosition(lf*(int)SKYSTONEConstants.TICKS_PER_INCH);
        myRobot.rf.setTargetPosition(rf*(int)SKYSTONEConstants.TICKS_PER_INCH);
        myRobot.lb.setTargetPosition(lb*(int)SKYSTONEConstants.TICKS_PER_INCH);
        myRobot.rb.setTargetPosition(rb*(int)SKYSTONEConstants.TICKS_PER_INCH);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && time.milliseconds()<4000 && opModeIsActive()){
            Log.d("Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void encoderStraightDrive(double inches, double power){
        encoderStraightDriveNoStop(inches, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void encoderStraightDriveNoStop(double inches, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStraightDriveNoReset(inches, power);
    }

    void encoderStraightDriveNoReset(double inches, double power){
        ElapsedTime time = new ElapsedTime();
        multiSetTargetPosition(inches* SKYSTONEConstants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && time.milliseconds()<4000 && opModeIsActive()){
            Log.d("Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
    }

    //Negative = Left, Positive = Right
    void encoderStrafeDriveInchesRight(double inches, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.lf.setTargetPosition((int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.lb.setTargetPosition(-(int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.rf.setTargetPosition(-(int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.rb.setTargetPosition((int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        ElapsedTime killTimer = new ElapsedTime();
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.lb, myRobot.rf, myRobot.rb) && opModeIsActive() && killTimer.seconds()<2){
            Log.d("SkyStone Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("SkyStone Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("SkyStone Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("SkyStone Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Positive = Clockwise, Negative = Counterclockwise
    void encoderTurn(double targetAngle, double power, double tolerance){
        encoderTurnNoStop(targetAngle, power, tolerance);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void encoderTurnNoStop(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, -power, power, tolerance, true);
    }

    void encoderTurnNoStopPowers(double targetAngle, double leftPower, double rightPower, double tolerance, boolean usePID) {
        double kR = SKYSTONEAutonomousConstants.kR;
        double kD = SKYSTONEAutonomousConstants.kD;

        //Undefined constants
        double d;
        double dt;
        double leftProportionalPower;
        double rightProportionalPower;
        //Initial error
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        double previousError = error;
        //Initial Time
        ElapsedTime clock = new ElapsedTime();
        double t1 = clock.nanoseconds();
        double t2 = t1;
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(error)>tolerance && opModeIsActive()){

            //Getting Error
            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle-currentAngle);
            if(usePID){
                //Getting time difference
                t2 = clock.nanoseconds();
                dt = t2-t1;

                //Setting d action
                d = (error-previousError)/dt*Math.pow(10,9);
                //Setting p action
                leftProportionalPower = Math.max(Math.min(error*kR + d*kD, 1),-1)*leftPower;
                rightProportionalPower = Math.max(Math.min(error*kR + d*kD, 1),-1)*rightPower;
                Log.d("Skystone: ", "leftProportionalPower: " + leftProportionalPower + " rightProportionalPower: " + rightProportionalPower);
                Log.d("Skystone: ", "dt: " + dt + "DerivativeAction: " + d*kD);
            }
            else{
                leftProportionalPower = leftPower*Math.signum(error);
                rightProportionalPower = rightPower*Math.signum(error);
            }

            //Set real power
            double realLeftPower = Math.max(Math.abs(leftPower/2), Math.abs(leftProportionalPower))*Math.signum(leftProportionalPower);
            double realRightPower = Math.max(Math.abs(rightPower/2), Math.abs(rightProportionalPower))*Math.signum(rightProportionalPower);
            runMotors(realLeftPower, realRightPower);

            //Store old values
            previousError = error;
            if(usePID){
                t1 = t2;
            }


            //Logging
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + realLeftPower + "rightPower: " + realRightPower + "CurrentAngle: " + currentAngle);
        }
    }

    void turn(double targetAngle, double leftPower, double rightPower, double tolerance){
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        ElapsedTime killTimer = new ElapsedTime();
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        while(Math.abs(error)>tolerance && opModeIsActive() && killTimer.seconds()<3){
            currentAngle = getHorizontalAngle();
            error = targetAngle-currentAngle;
            error = loopAround(error);
            runMotors(leftPower, rightPower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + leftPower + "rightPower: " + rightPower + "CurrentAngle: " + currentAngle);
        }
    }

    void encoderTurnNoStopLeftOnly(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, -power, 0, tolerance, true);
    }

    void encoderTurnNoStopRightOnly(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, 0, power, tolerance, true);
    }

    int leftFrontEncoder(){
        return myRobot.lf.getCurrentPosition();
    }

    int leftBackEncoder(){
        return myRobot.lb.getCurrentPosition();
    }

    int rightFrontEncoder(){
        return myRobot.rf.getCurrentPosition();
    }

    int rightBackEncoder(){
        return myRobot.rb.getCurrentPosition();
    }

    void straighteningEncoderDrive(double inches, double targetAngle, double tolerance, double power){
        straighteningEncoderDriveNoStop(inches, targetAngle, tolerance, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void straighteningEncoderDriveNoStop(double inches, double targetAngle, double tolerance, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        straighteningEncoderDriveNoReset(inches, targetAngle, tolerance, power);
    }

    void straighteningEncoderDriveNoReset(double inches, double targetAngle, double tolerance, double power){
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runMotors(power, power);
        double curDistance = leftFrontEncoder();
        double targetDistance = inches*SKYSTONEConstants.TICKS_PER_INCH;
        //double startDistance = curDistance;
        double errorDistance = targetDistance-curDistance;
        double adjust;
        while (Math.abs(errorDistance)>tolerance && opModeIsActive()){
            double currentAngle = getHorizontalAngle();
            double steer = getCorrection(currentAngle, targetAngle);
            adjust = Math.max(Math.min(errorDistance, 40),-40)/40*power;
            double leftSpeed = adjust - steer;
            double rightSpeed = adjust + steer;
            errorDistance = targetDistance-curDistance;
            runMotors(leftSpeed, rightSpeed);
            curDistance = leftFrontEncoder();
            Log.d("Skystone: ", "DistanceDrive Error: " + errorDistance +
                    " Angle Steer: " + steer + "CurrentDistance: " + curDistance + "Power: " + adjust + "Angle" + currentAngle);
        }
    }

    void adaptiveEncoderDrive(double inches, double targetAngle, double tolerance, double power){
        double fastMode;
        if(Math.abs(inches)>30){
            fastMode = (Math.abs(inches)-15)*Math.signum(inches);
            straighteningEncoderDriveNoStop(fastMode, targetAngle, tolerance, power);
        }
        encoderStraightDriveNoReset(inches, power);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getCorrection(double currentAngle, double targetAngle){
        double errorAngle = loopAround(targetAngle-currentAngle);
        double PCoefficient = 1.0/turnPValue;
        return errorAngle*PCoefficient;
    }

    void distanceEncoderDrive(double distance, double tolerance, double power, double targetAngle, DistanceSensor sensor){
        distanceEncoderDriveNoStop(distance, tolerance, power, targetAngle, sensor);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void distanceEncoderDriveNoStop(double distance, double tolerance, double power, double targetAngle, DistanceSensor sensor) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runMotors(power, power);
        double curDistance = sensor.getDistance(DistanceUnit.INCH);
        //double startDistance = curDistance;
        double error = curDistance-distance;
        double adjust;
        while (Math.abs(error)>tolerance && opModeIsActive()){
            /*
            adjust = 0.1*error/Math.abs(error) + 0.9*(Math.min(Math.abs(error),10))/10*power;
            if(error*power<0){
                adjust*=-1;
            }
             */
            double curAngle = getHorizontalAngle();
            //Get correction
            double steer = getCorrection(curAngle, targetAngle);
            //Get error
            error = curDistance-distance;
            //Once below 20 inches away, start slowing
            adjust = Math.max(Math.min(error, SKYSTONEAutonomousConstants.reducePowerDistance),-SKYSTONEAutonomousConstants.reducePowerDistance)/SKYSTONEAutonomousConstants.reducePowerDistance*power;
            //Ensure power is above 0.2
            adjust = floorPower(adjust);
            runMotors(adjust - steer, adjust + steer);
            curDistance = sensor.getDistance(DistanceUnit.INCH);
            Log.d("Skystone: ", "FrontDistanceDrive Error: " + error + " Adjust: " + adjust + "CurrentDistance: " + curDistance + "Steer: " + steer);
        }
    }

    private double floorPower(double power) {
        if(power > 0){
            power = Math.max(power, SKYSTONEAutonomousConstants.flooringPower);
        }
        else{
            power = Math.min(power, -SKYSTONEAutonomousConstants.flooringPower);
        }
        return power;
    }

    private double floorPower(double power, double floor) {
        if (Math.abs(power) < 0.00001){
            return power;
        }
        if(power > 0){
            power = Math.max(power, floor);
        }
        else{
            power = Math.min(power, -floor);
        }
        return power;
    }

    //private void
    //Shortcuts
    void setModeAllDrive(DcMotor.RunMode mode){
        myRobot.lb.setMode(mode);
        myRobot.lf.setMode(mode);
        myRobot.rb.setMode(mode);
        myRobot.rf.setMode(mode);
    }

    private void multiSetTargetPosition(double ticks, DcMotor...motors){
        for(DcMotor motor:motors){
            motor.setTargetPosition((int) Math.round(ticks));
        }
    }

    private boolean notCloseEnough(int tolerance, DcMotor...motors){
        for(DcMotor motor : motors){
            if(Math.abs(motor.getCurrentPosition()-motor.getTargetPosition()) > tolerance){
                return true;
            }
        }
        return false;
    }

    void runMotors (double leftPower, double rightPower){
        myRobot.lb.setPower(leftPower);
        myRobot.lf.setPower(leftPower);
        myRobot.rb.setPower(rightPower);
        myRobot.rf.setPower(rightPower);
    }

    //IMU Stuff
    double getHorizontalAngle(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.firstAngle;
        output = loopAround(output);
        return output;
    }

    private double loopAround(double output) {
        if (output > 180) {
            output -= 360;
        }
        if (output < -180) {
            output += 360;
        }
        return output;
    }

    double getRoll(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.secondAngle;
        output = loopAround(output);
        return output;
    }

    double getVerticalAngle(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.thirdAngle;
        output = loopAround(output);
        return output;
    }

    boolean opModeStatus(){
        return opModeIsActive();
    }

    void directionalDrive(double targetX, double targetY, boolean PID, double tolerance, double targetAngle){

        double maxVelocity = 1;
        double velocity = maxVelocity;
        double rotationVelocity = 0;
        double integral = 0;

        double currentAngle;
        double currentError;

        double dt;

        double t1 = 0;
        double xDis = 0;
        double yDis = 0;
        double totalDistance;
        RevBulkData prevData = myRobot.expansionHub.getBulkInputData();
        ElapsedTime clock = new ElapsedTime();

        //Check if movement is needed
        if(opModeIsActive()){
            //Determine distance and log it
            t1 = clock.nanoseconds();
            xDis = targetX - localizer.getX();
            yDis = targetY - localizer.getY();
            Log.d("Skystone: ",
                    "Direct Drive Target:(" + targetX + "," + targetY
                            + " tolerance" + tolerance + " Start At: ("
                            + localizer.getX() + "," + localizer.getY() + ")");

        }

        //While movement is needed
        while((Math.abs(yDis)>tolerance || Math.abs(xDis)>tolerance) && opModeIsActive()){
            //Kinematics Ratio with Encoder
            double strafeRatio = 0.9;
            double straightRatio = 1.1;
            //Defining constants
            double xI = 0.003;
            double yI = 0.003;
            double kR;
            double kP;
            double kD;
            double kI;
            //Setting constants
            //If using only encoders (backup option)
            if(localizer.getEncoderOnly()){
                kR = SKYSTONEAutonomousConstants.ekR;
                kP = SKYSTONEAutonomousConstants.ekP;
                kD = SKYSTONEAutonomousConstants.ekD;
                kI = 0.01;
            }
            //If using distance sensors (main option)
            else{
                kR = SKYSTONEAutonomousConstants.ddkR;
                kP = SKYSTONEAutonomousConstants.ddkP;
                kD = SKYSTONEAutonomousConstants.ddkD;
                kI = 0.01;
            }
            //Start of update
            //create a BulkData instance to more quickly display encoder data
            RevBulkData encoderData = myRobot.expansionHub.getBulkInputData();
            //Calculate time elapsed since last update
            double t2 = clock.nanoseconds();
            dt = t2-t1;
            //Calculate the current error in angle
            currentAngle = loopAround(localizer.getAngle());
            currentError = targetAngle - currentAngle;
            //Determine what to plug in as "rotation velocity" by multiplying error by the constant kR
            rotationVelocity = currentError * kR;
            //Update data (encoders, gyro, distance sensor etc)
            localizer.update(prevData, encoderData);
            //Add stats to telemetry, if any stats are available
            if (dashboard != null && localizer.ALPIP.size() != 0) {
                //Add the ratios of how far the robot needs to move on the x axis compared to the y one
                packet.put("xRatio", localizer.ALPIP.get(localizer.ALPIP.size() - 1).rX);
                packet.put("yRatio", localizer.ALPIP.get(localizer.ALPIP.size() - 1).rY);
                //Add the current position of the robot on the coordinate system/
                packet.put("x", localizer.getX());
                packet.put("y", localizer.getY());
                //packet.fieldOverlay().setFill("black").fillCircle(x, y, 1);
                dashboard.sendTelemetryPacket(packet);
            }

            //End of update
            xDis = targetX - localizer.getX();
            yDis = targetY - localizer.getY();

            Log.d("Skystone:", "GyroError: " + rotationVelocity);
            double angle;
            //If using PID loop
            if(PID) {
                //Determine total distance from target position
                totalDistance = Math.sqrt(xDis*xDis+yDis*yDis);
                //double dXDis = (xRaw-previousX)/dt*Math.pow(10,9);
                //double dYDis = (yRaw-previousY)/dt*Math.pow(10,9);

                //Determine the derivative value (multiplies velocity of motor by derivative constant)
                //Note: Always uses encoders due to slow update rate and occasional spikes with distance sensors.
                double dY = localizer.getdY(encoderData);
                double dX = localizer.getdX(encoderData);

                //Update the integral value by adding the integral value of the current update to the total integral
                //Change in time * change in distance
                xI += dt*xDis;
                yI += dt*yDis;
                //Determine the velocities based on P, I and D inputs
                double xVelocity = maxVelocity * getPID(xDis, dX, xI, kD, kP, kI) * strafeRatio;
                double yVelocity = maxVelocity * getPID(yDis, dY, yI, kD, kP, kI) * straightRatio;
                //Convert these velocities into an angle that can be taken by free drive
                angle = Math.atan2(xVelocity * SKYSTONEAutonomousConstants.lateralFactor, yVelocity);

                //Cap max velocity at one
                velocity = Math.min (1, /*SKYSTONEAutonomousConstants.minimumPower +*/Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity));

                //Log everything
                Log.d("Skystone", "Distance error = " + totalDistance + " Change in xDistance error = " + dX + " Change in yDistance error = " + dY + " dt = " + dt);
                Log.d("Skystone:", " Proportional Action = " + totalDistance*kP + " xVelocity = " + xVelocity + " yVelocity = " + yVelocity);
                Log.d("Skystone", " kP: " + kP + "kD: " + kD + " kR: " + kR);
                Log.d("IAX", "Time: " + clock.seconds() + " XError: " + xDis + " kP: " + kP + " kD: " +
                        kD + " xVelocity: " + xVelocity + " pxChange: " + xDis*kP + " dxChange: " + dX*kD + "xIntegral: " + xI);
                Log.d("IAY", "Time: " + clock.seconds() + " YError: " + yDis + " kP: " + kP + " kD: " +
                        kD + " yVelocity: " + yVelocity + " pyChange: " + yDis*kP + " dyChange: " + dY*kD + "yIntegral: " + yI);
            }
            //If not using PID
            else{
                //Determine angle based on distance alone.
                angle = Math.atan2(xDis * SKYSTONEAutonomousConstants.lateralFactor ,yDis);
            }
            //Create a variable to store end time of this update/start time of next update
            t1=t2;

            //Adjust based on which corner of the field the robot is in.
            if(localizer.getCorner() == Localizer.Corner.RIGHT_UP
                    || localizer.getCorner() == Localizer.Corner.RIGHT_DOWN){
                angle = Math.PI+angle;
            }
            //Drive based on the determined angle, velocity and rotation velocity.
            freeDrive(angle, velocity, rotationVelocity);
            //Log the angle, velocity and rotation velocity
            Log.d("Skystone: ", "Skystone Angle: "+ (angle*180/Math.PI) + "Velocity: " + velocity+ " RotationVelocity" + rotationVelocity);
            prevData = encoderData;
        }
    }

    double getP(double error, double kP){
        return Math.min(1,Math.abs(error)*kP);
    }

    double getPID(double error, double dError, double I, double kD, double kP, double kI){
        //Error * Proportion constant + Change in Error (Derivative of Error vs Time) * Derivative Constant + Total Distance Traveled (Integral of Error vs Time) * Integral Constant
        return error*kP + dError*kD + I*kI;
    }

    void freeDrive(double direction, double velocity, double rotationVelocity){
        double s = Math.sin(direction + Math.PI / 4.0);
        double c = Math.cos(direction + Math.PI / 4.0);
        double a = Math.max(Math.abs(s), Math.abs(c));
        s /= a;
        c /= a;

        final double v1 = velocity * s + rotationVelocity;
        final double v2 = velocity * c - rotationVelocity;
        final double v3 = velocity * c + rotationVelocity;
        final double v4 = velocity * s - rotationVelocity;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);
        myRobot.lf.setPower(v1/scale);
        myRobot.rf.setPower(v2/scale);
        myRobot.lb.setPower(v3/scale);
        myRobot.rb.setPower(v4/scale);
        Log.d("Skystone: ", " lfPower: " + v1/scale + " rfPower: " + v2/scale + "lbPower: " + v3/scale + " rbPower: " + v4/scale);
    }
}
