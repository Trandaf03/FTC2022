package org.firstinspires.ftc.teamcode.robotComponents;

import static org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo.COUNTS_PER_CM;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.robotInfo;
import org.firstinspires.ftc.teamcode.utilities.autonomousDriveUtilities.Gyro;

import org.firstinspires.ftc.teamcode.utilities.driveUtilities.encoderUsing;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.powerBehavior;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotDirection;
import org.firstinspires.ftc.teamcode.utilities.driveUtilities.robotStopping;

public class driveComponents {


    public enum ROBOT_MOVEMENT{
        FORWARD_DIR, REVERSE_DIR, LEFT_DIR, RIGHT_DIR, SPLINE_TO_DIR, ROTATE_LEFT, ROTATE_RIGHT;
    }

    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    private Gyro gyro = new Gyro();
    private robotInfo robot = new robotInfo();

    powerBehavior motorBreaking = new powerBehavior();
    robotStopping stopping = new robotStopping();
    robotDirection robotDir = new robotDirection();
    encoderUsing encoders = new encoderUsing();

    private static final double     COUNTS_PER_MOTOR_REV    = 386.3 ;
    private static PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);
    private PIDCoefficients v1pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v2pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v3pidGains = new PIDCoefficients(0,0,0);
    private PIDCoefficients v4pidGains = new PIDCoefficients(0,0,0);
    ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private double v1IntegralPower = 0;
    private double v2IntegralPower = 0;
    private double v3IntegralPower = 0;
    private double v4IntegralPower = 0;

    private double v1LastError = 0;
    private double v2LastError = 0;
    private double v3LastError = 0;
    private double v4LastError = 0;

    public double startHeading = gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.HEADING);

    public driveComponents(){

    }


    public void driveInitialization(HardwareMap map,
                                    robotDirection.ROBOT_DIRECTIONS heading,
                                    powerBehavior.ROBOT_BREAKING breakingMode,
                                    encoderUsing.ENCODER_RUNNING_MODE encoderMode){
        leftFront = map.get(DcMotorEx.class, "leftFront");
        rightFront = map.get(DcMotorEx.class, "rightFront");
        leftRear = map.get(DcMotorEx.class, "leftRear");
        rightRear = map.get(DcMotorEx.class, "rightRear");


        robotDir.setMotorsName(leftFront,leftRear,rightFront,rightRear);
        motorBreaking.setMotorsName(leftFront,leftRear,rightFront,rightRear);
        encoders.setMotorsName(leftFront,leftRear,rightFront,rightRear);

        robotDir.setRobotDirection(heading);
        motorBreaking.setBreakingMode(breakingMode);
        encoders.setEncoderMode(encoderMode);

        stopping.driveStop(leftFront,leftRear,rightFront,rightRear);
    }

    public void robotVelocityController(double left_stick_x, double left_stick_y, double right_stick_x){
        double r = Math.hypot(left_stick_x, -left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, -left_stick_x) - Math.PI / 4;
        double rightX = -right_stick_x;
        final double v1 = (r * Math.cos(robotAngle) + rightX) * 386.3 * 20;
        final double v2 = (r * Math.sin(robotAngle) - rightX) * 386.3 * 20;
        final double v3 = (r * Math.sin(robotAngle) + rightX) * 386.3 * 20;
        final double v4 = (r * Math.cos(robotAngle) - rightX) * 386.3 * 20;

        leftFront.setVelocity(v1);
        rightFront.setVelocity(v2);
        leftRear.setVelocity(v3);
        rightRear.setVelocity(v4);

    }


    public void stop(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    public void moveRobot(double nextX, double nextY, double speed) throws InterruptedException {

        //startHeading = gyro.returnAngle(Gyro.ROBOT_GYRO_DIRECTION.HEADING);

        if(nextX != 0 && nextY == 0){
            if(nextX > 0){
                moveToPosition(ROBOT_MOVEMENT.FORWARD_DIR,nextX,nextY,speed);
            } else {
                moveToPosition(ROBOT_MOVEMENT.REVERSE_DIR,nextX,nextY,speed);
            }
        }
        if(nextX == 0 && nextY != 0){
            nextX *= 1.67;
            nextY *= 1.67;
            if( nextY > 0){
                moveToPosition(ROBOT_MOVEMENT.RIGHT_DIR,nextX,nextY,speed);
            } else {
                moveToPosition(ROBOT_MOVEMENT.LEFT_DIR,nextX,nextY,speed);
            }
        }
        //if(nextX != 0 && nextY != 0){
            //moveToPosition(ROBOT_MOVEMENT.SPLINE_TO_DIR,nextX,nextY,speed);
        //}
    }
    private void moveToPosition(ROBOT_MOVEMENT movement_direction, double xDistance, double yDistance, double speed) throws InterruptedException {
        switch (movement_direction){
            case FORWARD_DIR:
                xMovement(-xDistance,speed);
                break;

            case REVERSE_DIR:
                xMovement(xDistance,speed);
                break;

            case LEFT_DIR:
                yMovement(-yDistance,speed);
                break;

            case RIGHT_DIR:
                yMovement(yDistance,speed);
                break;

            case SPLINE_TO_DIR:
                spline(-xDistance, yDistance, speed);
                break;
            default:
                break;

        }
    }
    private void xMovement(double distance, double speed) throws InterruptedException {
        setMotorsEnabled();

        distance *= COUNTS_PER_CM;
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);

        encoders.setTargetPositionXmovement((int)distance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        setRobotMotorsPower(speed);

        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            PIDmovement(setMotorPower(speed));
        }

       setMotorsDisabled();

    }
    private void yMovement(double distance, double speed) throws InterruptedException {
        setMotorsEnabled();

        distance *= COUNTS_PER_CM;
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);

        encoders.setTargetPositionYmovement((int)distance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        setRobotMotorsPower(speed);

        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            PIDmovement(setMotorPower(speed));
        }

        setMotorsDisabled();

    }
    private void spline(double xDistance, double yDistance, double speed) throws InterruptedException {
        if(xDistance > 0 && yDistance > 0){
            splineToFrontRight(xDistance,yDistance,speed);
        }
        if(xDistance > 0 && yDistance < 0){
            splineToFrontLeft(xDistance,yDistance,speed);
        }
        if(xDistance < 0 && yDistance > 0){
            splineToBackRight(xDistance,yDistance,speed);
        }
        if(xDistance < 0 && yDistance < 0){
            splinetoBackLeft(xDistance,yDistance,speed);
        }

    }
    private void splineToFrontRight(double xDistance, double yDistance, double speed) throws InterruptedException{
        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;
        // leftfront and backright;
        Thread.sleep(100);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Thread.sleep(100);
        leftFront.setTargetPosition(-(int)distance);
        rightRear.setTargetPosition(-(int)distance);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFront.setPower(speed);
        rightRear.setPower(speed);

        while (leftFront.isBusy() && rightRear.isBusy()){
            leftFront.setPower(speed);
            rightRear.setPower(speed);
        }

        leftFront.setPower(0);
        rightRear.setPower(0);

    }
    private void splineToFrontLeft(double xDistance, double yDistance, double speed) throws InterruptedException{
        // frontright and leftback;
        double distance = Math.hypot(xDistance,yDistance) * COUNTS_PER_CM;
        Thread.sleep(100);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Thread.sleep(100);
        leftRear.setTargetPosition((int)distance);
        rightFront.setTargetPosition((int)distance);

        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftRear.setPower(speed);
        rightFront.setPower(speed);

        while (leftRear.isBusy() && rightFront.isBusy()){

        }

        leftRear.setPower(0);
        rightFront.setPower(0);
    }
    private void splineToBackRight(double xDistance, double yDistance, double speed) throws InterruptedException {
        spline(-xDistance,-yDistance,speed);
    }
    private void splinetoBackLeft(double xDistance, double yDistance, double speed) throws InterruptedException {
        splineToFrontRight(-xDistance,-yDistance,speed);
    }

    public void testSpline(double xDistance, double yDistance, double speed){
        setMotorsEnabled();


        xDistance = xDistance * COUNTS_PER_CM;
        yDistance = yDistance * COUNTS_PER_CM;
        double XYHypot = Math.hypot(xDistance,yDistance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.STOP_AND_RESET);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.RUN_USING);

        //encoders.setTargetPositionXmovement((int)distance);
        encoders.splineSetTargetPosition1((int)yDistance);
        encoders.splineSetTargetPostion2((int)xDistance);
        encoders.setEncoderMode(encoderUsing.ENCODER_RUNNING_MODE.TO_POSITION);

        //setRobotMotorsPower(speed);
        //yMovement
        //leftRear.setPower(1);
        //rightFront.setPower(1);
        //xMovement
        //leftFront.setPower(1);
        //rightRear.setPower(1);
        double robotXCurrentPosition = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2;
        double robotYCurrentPosition = (leftRear.getCurrentPosition() + rightFront.getCurrentPosition())/2;
        double currentXYHyport = Math.hypot(robotXCurrentPosition,robotYCurrentPosition);

        do{
            leftRear.setPower(Math.abs(1-valueTo1(currentXYHyport,0,XYHypot,0,1)));
            rightFront.setPower(Math.abs(1-valueTo1(currentXYHyport,0,XYHypot,0,1)));

            leftFront.setPower(Math.abs(valueTo1(currentXYHyport,0,XYHypot,0,1)));
            rightRear.setPower(-Math.abs(valueTo1(currentXYHyport,0,XYHypot,0,1)));

            robotXCurrentPosition = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2;
            robotYCurrentPosition = (leftRear.getCurrentPosition() + rightFront.getCurrentPosition())/2;
            currentXYHyport = Math.hypot(robotXCurrentPosition,robotYCurrentPosition);

        }while(currentXYHyport < XYHypot);

       setMotorsDisabled();

    }

    private void PIDmovement(double speed){

        pidTimer.reset();
        final double v1TargetVelocity = setMotorPower(speed);
        final double v2TargetVelocity = setMotorPower(speed);
        final double v3TargetVelocity = setMotorPower(speed);
        final double v4TargetVelocity = setMotorPower(speed);

        double v1CurrentVelocity = leftFront.getVelocity();
        double v2CurrentVelocity = rightFront.getVelocity();
        double v3CurrentVelocity = leftRear.getVelocity();
        double v4CurrentVelocity = rightRear.getVelocity();

        double v1Error = v1TargetVelocity - v1CurrentVelocity;
        double v2Error = v1TargetVelocity - v2CurrentVelocity;
        double v3Error = v1TargetVelocity - v3CurrentVelocity;
        double v4Error = v1TargetVelocity - v4CurrentVelocity;

        v1IntegralPower += v1Error * pidTimer.time();
        v2IntegralPower += v2Error * pidTimer.time();
        v3IntegralPower += v3Error * pidTimer.time();
        v4IntegralPower += v4Error * pidTimer.time();

        double v1deltaError = v1Error - v1LastError;
        double v2deltaError = v1Error - v2LastError;
        double v3deltaError = v1Error - v3LastError;
        double v4deltaError = v1Error - v4LastError;

        double v1Derivative = v1deltaError / pidTimer.time();
        double v2Derivative = v2deltaError / pidTimer.time();
        double v3Derivative = v3deltaError / pidTimer.time();
        double v4Derivative = v4deltaError / pidTimer.time();

        v1pidGains.p = pidCoefficients.p * v1Error;
        v2pidGains.p = pidCoefficients.p * v2Error;
        v3pidGains.p = pidCoefficients.p * v3Error;
        v4pidGains.p = pidCoefficients.p * v4Error;

        v1pidGains.i = pidCoefficients.i * v1IntegralPower;
        v2pidGains.i = pidCoefficients.i * v2IntegralPower;
        v3pidGains.i = pidCoefficients.i * v3IntegralPower;
        v4pidGains.i = pidCoefficients.i * v4IntegralPower;

        v1pidGains.d = pidCoefficients.d * v1Derivative;
        v2pidGains.d = pidCoefficients.d * v2Derivative;
        v3pidGains.d = pidCoefficients.d * v3Derivative;
        v4pidGains.d = pidCoefficients.d * v4Derivative;

        leftFront.setVelocity(v1pidGains.p + v1pidGains.i + v1pidGains.d + v1TargetVelocity);
        rightFront.setVelocity(v2pidGains.p + v2pidGains.i + v2pidGains.d + v2TargetVelocity);
        leftRear.setVelocity(v3pidGains.p + v3pidGains.i + v3pidGains.d + v3TargetVelocity);
        rightRear.setVelocity(v4pidGains.p + v4pidGains.i + v4pidGains.d + v4TargetVelocity);

        v1LastError = v1Error;
        v2LastError = v2Error;
        v3LastError = v3Error;
        v4LastError = v4Error;
    }

    public double setMotorPower(double power){
        return power * COUNTS_PER_MOTOR_REV;
    }
    public void setRobotMotorsPower(double speed){
        leftFront.setVelocity(setMotorPower(speed));
        rightFront.setVelocity(setMotorPower(speed));
        leftRear.setVelocity(setMotorPower(speed));
        rightRear.setVelocity(setMotorPower(speed));
    }
    private void setMotorsEnabled(){
        leftFront.setMotorEnable();
        rightFront.setMotorEnable();
        leftRear.setMotorEnable();
        rightRear.setMotorEnable();
    }
    private void setMotorsDisabled(){
        leftFront.setMotorDisable();
        rightFront.setMotorDisable();
        leftRear.setMotorDisable();
        rightRear.setMotorDisable();
    }

    double valueTo1(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }




}
